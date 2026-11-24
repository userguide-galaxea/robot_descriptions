cmake_minimum_required(VERSION 2.8.3)

project(a1z_g1z_camera_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config docs launch meshes preview urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
		PATTERN "__pycache__" EXCLUDE
		PATTERN "*.pyc" EXCLUDE
		PATTERN ".gitignore" EXCLUDE)
endforeach(dir)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
