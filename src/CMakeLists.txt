rock_library(envire_urdf
    SOURCES GraphLoader.cpp
    HEADERS GraphLoader.hpp
            EnvireLoader.hpp
    DEPS_PKGCONFIG envire_core urdfdom)

rock_executable(envire_urdf_bin Main.cpp
    DEPS envire_urdf)

