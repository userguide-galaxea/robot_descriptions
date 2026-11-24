// Local CPU ray/triangle intersections for the ROS optical preview.
// Binary STL meshes retain their original geometry; BVHs stay in link frames.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <numeric>
#include <thread>
#include <vector>

struct V {
    float x, y, z;
    float operator[](int a) const { return a == 0 ? x : (a == 1 ? y : z); }
    V operator+(V b) const { return {x+b.x, y+b.y, z+b.z}; }
    V operator-(V b) const { return {x-b.x, y-b.y, z-b.z}; }
    V operator*(float s) const { return {x*s, y*s, z*s}; }
};
static float dot(V a, V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static V cross(V a, V b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
struct Tri { V p, e1, e2, normal, center; };
struct Box {
    V lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    void add(V p) {
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    bool hit(V o, V d, float limit) const {
        float near=.0001f, far=limit;
        for (int a=0;a<3;++a) {
            if(std::abs(d[a])<1e-12f) { if(o[a]<lo[a] || o[a]>hi[a]) return false; }
            else {float t1=(lo[a]-o[a])/d[a],t2=(hi[a]-o[a])/d[a];
                if(t1>t2)std::swap(t1,t2);
                near=std::max(near,t1);far=std::min(far,t2);if(near>far)return false;}
        }
        return true;
    }
};
struct Node { Box box; int start=0,count=0,left=-1,right=-1; };
struct Mesh {
    std::vector<Tri> tris;
    std::vector<int> order;
    std::vector<Node> nodes;
    int build(int start,int count) {
        int idx=static_cast<int>(nodes.size());nodes.emplace_back();
        Box bounds,centers;
        for(int i=start;i<start+count;++i) {
            const auto& t=tris[order[i]];bounds.add(t.p);bounds.add(t.p+t.e1);bounds.add(t.p+t.e2);centers.add(t.center);
        }
        nodes[idx].box=bounds;
        if(count<=8) {nodes[idx].start=start;nodes[idx].count=count;return idx;}
        V span=centers.hi-centers.lo;int axis=span.y>span.x?1:0;if(span.z>span[axis])axis=2;
        int mid=start+count/2;
        std::nth_element(order.begin()+start,order.begin()+mid,order.begin()+start+count,
                         [&](int a,int b){return tris[a].center[axis]<tris[b].center[axis];});
        int left=build(start,mid-start),right=build(mid,start+count-mid);
        nodes[idx].left=left;nodes[idx].right=right;return idx;
    }
    bool intersect(V o,V d,float& nearest,V& normal,float minimum=.0001f) const {
        if(nodes.empty())return false;
        bool found=false;int stack[128],top=0;stack[top++]=0;
        while(top) {
            const auto& node=nodes[stack[--top]];
            if(!node.box.hit(o,d,nearest))continue;
            if(node.count) {
                for(int i=node.start;i<node.start+node.count;++i) {
                    const auto& t=tris[order[i]];
                    V pvec=cross(d,t.e2);float det=dot(t.e1,pvec);
                    if(std::abs(det)<1e-14f)continue;
                    float inv=1.f/det;V tvec=o-t.p;float u=dot(tvec,pvec)*inv;
                    if(u<0 || u>1)continue;
                    V qvec=cross(tvec,t.e1);float v=dot(d,qvec)*inv;
                    if(v<0 || u+v>1)continue;
                    float distance=dot(t.e2,qvec)*inv;
                    if(distance>minimum && distance<nearest) {nearest=distance;normal=t.normal;found=true;}
                }
            } else {stack[top++]=node.left;stack[top++]=node.right;}
        }
        return found;
    }
};
extern "C" {
void* mesh_load(const char* path) {
    std::ifstream file(path,std::ios::binary);if(!file)return nullptr;
    file.seekg(80);uint32_t count=0;file.read(reinterpret_cast<char*>(&count),4);
    if(!file || count>10000000)return nullptr;
    auto* m=new Mesh();m->tris.reserve(count);
    for(uint32_t i=0;i<count;++i) {
        float data[12];uint16_t attr;
        file.read(reinterpret_cast<char*>(data),48);file.read(reinterpret_cast<char*>(&attr),2);
        if(!file) {delete m;return nullptr;}
        V p{data[3],data[4],data[5]},e1=V{data[6],data[7],data[8]}-p,e2=V{data[9],data[10],data[11]}-p;
        V normal=cross(e1,e2);float len=std::sqrt(dot(normal,normal));if(len<1e-16f)continue;
        m->tris.push_back({p,e1,e2,normal*(1.f/len),p+(e1+e2)*(1.f/3.f)});
    }
    m->order.resize(m->tris.size());std::iota(m->order.begin(),m->order.end(),0);
    if(!m->tris.empty())m->build(0,static_cast<int>(m->tris.size()));
    return m;
}
void mesh_free(void* ptr) {delete static_cast<Mesh*>(ptr);}
// Rays are unit vectors in base_link. Matrices are row-major base_T_link.
// distances contains existing scene depths and is updated with nearest hits.
void mesh_occlude_near(void** handles,const float* matrices,int mesh_count,
                  const float* origin,const float* rays,int ray_count,
                  float* distances,uint8_t* rgb,uint8_t* mask,const float* minimums) {
    struct Instance {const Mesh* mesh;V origin;std::array<float,16> T;float minimum;};
    std::vector<Instance> instances;
    for(int m=0;m<mesh_count;++m) {
        const float* T=matrices+m*16;V delta{origin[0]-T[3],origin[1]-T[7],origin[2]-T[11]};
        V local{T[0]*delta.x+T[4]*delta.y+T[8]*delta.z,
                T[1]*delta.x+T[5]*delta.y+T[9]*delta.z,
                T[2]*delta.x+T[6]*delta.y+T[10]*delta.z};
        Instance in{static_cast<Mesh*>(handles[m]),local,{},minimums ? minimums[m] : .0001f};std::copy(T,T+16,in.T.begin());instances.push_back(in);
    }
    auto work=[&](int first,int last) {
        for(int i=first;i<last;++i) {
            V world{rays[i*3],rays[i*3+1],rays[i*3+2]};
            for(const auto& in:instances) {
                const auto& T=in.T;
                V d{T[0]*world.x+T[4]*world.y+T[8]*world.z,
                    T[1]*world.x+T[5]*world.y+T[9]*world.z,
                    T[2]*world.x+T[6]*world.y+T[10]*world.z},normal;
                if(in.mesh->intersect(in.origin,d,distances[i],normal,in.minimum)) {
                    V n{T[0]*normal.x+T[1]*normal.y+T[2]*normal.z,
                        T[4]*normal.x+T[5]*normal.y+T[6]*normal.z,
                        T[8]*normal.x+T[9]*normal.y+T[10]*normal.z};
                    float shade=.4f+.6f*std::abs(dot(n,V{.25916f,-.43193f,.86387f}));
                    auto c=static_cast<uint8_t>(std::min(255.f,220.f*shade));
                    rgb[i*3]=rgb[i*3+1]=rgb[i*3+2]=c;mask[i]=1;
                }
            }
        }
    };
    unsigned threads=std::min(8u,std::max(1u,std::thread::hardware_concurrency()));
    std::vector<std::thread> workers;
    for(unsigned t=0;t<threads;++t)workers.emplace_back(work,ray_count*t/threads,ray_count*(t+1)/threads);
    for(auto& t:workers)t.join();
}
// Preserve the original entry point and A1Z behavior.
void mesh_occlude(void** handles,const float* matrices,int mesh_count,
                  const float* origin,const float* rays,int ray_count,
                  float* distances,uint8_t* rgb,uint8_t* mask) {
    mesh_occlude_near(handles,matrices,mesh_count,origin,rays,ray_count,
                      distances,rgb,mask,nullptr);
}

}
