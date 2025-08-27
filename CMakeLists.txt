# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1)
project(envire_urdf VERSION 0.1)

find_package(Rock)
find_package(PluginManager)
find_package(Boost REQUIRED COMPONENTS system thread)

rock_init()
rock_feature(NOCURDIR)
rock_standard_layout()
