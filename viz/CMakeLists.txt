
rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

if(ROCK_QT_VERSION_4)
    rock_vizkit_plugin(envire_urdf-viz
        URDF_Visual.cpp
        DEPS envire_urdf
        MOC URDF_Visual.hpp
        HEADERS URDF_Visual.hpp)
endif(ROCK_QT_VERSION_4)

if(ROCK_QT_VERSION_5)
    # temporary check until vizkit3d-qt5 is actually available
    find_package(vizkit3d-qt5)
    if (vizkit3d-qt5_FOUND)
        rock_vizkit_plugin_qt5(envire_urdf-viz-qt5
            URDF_Visual.cpp
            DEPS envire_urdf
            MOC5 URDF_Visual.hpp
            HEADERS URDF_Visual.hpp)
    endif(vizkit3d-qt5_FOUND)
endif(ROCK_QT_VERSION_5)
