find_package(Boost)

rock_testsuite(test_suite suite.cpp
    test_robot.cpp
    DEPS envire_urdf
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG 
        envire_core)

target_link_libraries(test_suite ${Boost_LIBRARIES})
