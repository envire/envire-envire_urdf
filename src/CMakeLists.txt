rock_library(envire_urdf
    SOURCES GraphLoader.cpp
    HEADERS GraphLoader.hpp
    DEPS_PKGCONFIG envire_core urdfdom mars_interfaces base-lib)

rock_executable(envire_urdf_bin Main.cpp
    DEPS envire_urdf)

