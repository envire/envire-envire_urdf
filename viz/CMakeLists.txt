
rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX envire_urdf-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES URDF_Visual.cpp
    DEPS envire_urdf
    MOC URDF_Visual.hpp
    HEADERS URDF_Visual.hpp)

