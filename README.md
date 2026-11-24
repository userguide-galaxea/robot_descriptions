# Robot descriptions

Robot descriptions and their supporting meshes, launch files, and configuration.

ROS package names have changed. Update downstream launch files, package dependencies,
`package://` paths, and Isaac Sim ROS package mappings to use the new names.

| Previous ROS package | Current ROS package |
| --- | --- |
| `A1_URDF_0607_0028` | `a1_description` |
| `a1x` | `a1x_description` |
| `a1y` | `a1y_description` |
| `A1Z_Flange` | `a1z_flange_description` |
| `A1Z_G1Z` | `a1z_g1z_description` |
| `r1_v2_1_0` | `r1_description` |
| `r1pro_urdf` | `r1pro_g1z_2026_description` |
| `lemo_description` | `lemo_description` |

Robot Arm:
- [A1Z URDF](./a1z_description/)
- [A1Z G1Z camera, calibration and ROS 2 preview](./a1z_description/a1z_g1z_camera_description/)
- [A1X URDF](./a1x_description/)
- [A1Y URDF](./a1y_description/)
- [A1 URDF](./a1_description/)

Robot:
- [LEMO URDF](./lemo_description/)
- [R1 Pro 2026 URDF](./r1pro_description/r1pro_g1z_2026_description/)
- [R1 Lite 2026 URDF](./r1lite_description/urdf_r1lite_2026/)
- [R1 PRO 2025 URDF](./r1pro_description/urdf_r1pro_2025.urdf)
- [R1 Lite 2025 URDF](./r1lite_description/urdf_r1lite_2025.urdf)
- [R1 URDF](./r1_description/)