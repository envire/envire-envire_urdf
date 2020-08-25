#include <iostream>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include "URDF_Visual.hpp"

using namespace vizkit3d;

struct URDF_Visual::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    ::urdf::Visual data;
};


URDF_Visual::URDF_Visual()
    : p(new Data), mainNode(new osg::Group())
{
}

URDF_Visual::~URDF_Visual()
{
    delete p;
}

osg::ref_ptr<osg::Node> URDF_Visual::createMainNode()
{
    //we cannot read the content here, so we need to wait until we get the ::urdf::Visual data
    return mainNode;
}

void URDF_Visual::updateMainNode ( osg::Node* node )
{
    //osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
    if (mainNode->getNumChildren() == 0){

        //read color
        ::urdf::MaterialSharedPtr material = p->data.material;
        //use grey as default color
        osg::Vec4 color(0.2,0.2,0.2,1);
        if (material.get()){
            //looks like the parser already sets the color.
            color = osg::Vec4 (material->color.r,material->color.g,material->color.b,material->color.a);
            //TODO: load texture
        }
        switch (p->data.geometry->type){
            case ::urdf::Geometry::SPHERE:{
                    ::urdf::Sphere* sphere = dynamic_cast<::urdf::Sphere*>(p->data.geometry.get());
                    osg::ref_ptr<osg::Sphere> shape = new osg::Sphere(osg::Vec3(0,0,0),sphere->radius);
                    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(shape);
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
                    drawable->setColor(color);
                    geode->addDrawable(drawable);
                    geode->setName(p->data.name);
                    mainNode->addChild(geode);
                    break;
                }
            case ::urdf::Geometry::BOX:{
                    ::urdf::Box* box = dynamic_cast<::urdf::Box*>(p->data.geometry.get());
                    osg::ref_ptr<osg::Box> shape = new osg::Box(osg::Vec3(0,0,0),box->dim.x,box->dim.y,box->dim.z);
                    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(shape);
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
                    drawable->setColor(color);
                    geode->addDrawable(drawable);
                    geode->setName(p->data.name);
                    mainNode->addChild(geode);
                    break;
                }
            case ::urdf::Geometry::CYLINDER:{
                    ::urdf::Cylinder* cylinder = dynamic_cast<::urdf::Cylinder*>(p->data.geometry.get());
                    osg::ref_ptr<osg::Cylinder> shape = new osg::Cylinder(osg::Vec3(0,0,0),cylinder->radius,cylinder->length);
                    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(shape);
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
                    drawable->setColor(color);
                    geode->addDrawable(drawable);
                    geode->setName(p->data.name);
                    mainNode->addChild(geode);
                    break;
            }
            case ::urdf::Geometry::MESH:{
                ::urdf::Mesh* mesh = dynamic_cast<::urdf::Mesh*>(p->data.geometry.get());
                osg::ref_ptr<osg::Node> visual = osgDB::readNodeFile(mesh->filename);
                osg::ref_ptr<osg::Material> mat = new osg::Material;
                mat->setDiffuse (osg::Material::FRONT_AND_BACK, color); 

                osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
                mainNode->addChild(transform);
                transform->setScale(osg::Vec3d (mesh->scale.x, mesh->scale.y, mesh->scale.z));
                if (visual){
                    visual->setName(p->data.name);
                    visual->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE); 
                    transform->addChild(visual);
                }
                break;
            }
        }
    }
}

void URDF_Visual::updateDataIntern(::urdf::Visual const& value)
{
    p->data = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(URDF_Visual)

