#include <iostream>
#include "URDF_Visual.hpp"

using namespace vizkit3d;

struct URDF_Visual::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    ::urdf::Visual data;
};


URDF_Visual::URDF_Visual()
    : p(new Data)
{
}

URDF_Visual::~URDF_Visual()
{
    delete p;
}

osg::ref_ptr<osg::Node> URDF_Visual::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void URDF_Visual::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void URDF_Visual::updateDataIntern(::urdf::Visual const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(URDF_Visual)

