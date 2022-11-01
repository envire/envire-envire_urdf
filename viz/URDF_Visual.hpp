#ifndef envire_urdf_URDF_Visual_H
#define envire_urdf_URDF_Visual_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <urdf_model/link.h>

namespace vizkit3d
{
    class URDF_Visual
        : public vizkit3d::Vizkit3DPlugin<::urdf::Visual>
        , boost::noncopyable
    {
    Q_OBJECT

    public:
        URDF_Visual();
        ~URDF_Visual();

    Q_INVOKABLE void updateData(::urdf::Visual const &sample)
    {vizkit3d::Vizkit3DPlugin<::urdf::Visual>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(::urdf::Visual const& plan);
        
    private:
        struct Data;
        Data* p;
        osg::ref_ptr<osg::Group> mainNode;
    };

    //Macro that makes this plugin loadable in ruby, this is optional.
    VizkitQtPlugin(URDF_Visual)

}
#endif
