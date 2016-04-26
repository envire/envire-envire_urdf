#include "Robot.hpp"
#include <iostream>
#include <urdf_model/link.h>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

void envire::urdf::Robot::loadFromUrdf(const std::string& path)
{
    urdfModel = ::urdf::parseURDFFile(path);
}

void envire::urdf::Robot::initGraph(envire::core::EnvireGraph& graph)
{
    initialized = true;
    initFrames(graph);
    initTfs(graph);
}

void envire::urdf::Robot::initGraph(envire::core::EnvireGraph& graph, envire::core::GraphTraits::vertex_descriptor linkTo)
{
    if (!initialized)
    {
        initGraph(graph);
    }
    envire::core::FrameId robotRoot = urdfModel->getRoot()->name;
    iniPose.time = base::Time::now();
    graph.addTransform(graph.getFrameId(linkTo), robotRoot, iniPose);
}

void envire::urdf::Robot::initFrames(envire::core::EnvireGraph& graph)
{
    for(std::pair<std::string, boost::shared_ptr<::urdf::Link> > frame : urdfModel->links_)
    {
       graph.addFrame(frame.second->name);
    }
    
}

void envire::urdf::Robot::initTfs(envire::core::EnvireGraph& graph)
{
    for(std::pair<std::string, boost::shared_ptr<::urdf::Joint> > tf : urdfModel->joints_) 
    {
        Eigen::Affine3d tfPose(Eigen::Affine3d::Identity());
        ::urdf::Vector3 pos = tf.second->parent_to_joint_origin_transform.position;
        ::urdf::Rotation rot = tf.second->parent_to_joint_origin_transform.rotation;
        tfPose.linear() = Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z).matrix();
        tfPose.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);
        envire::core::Transform envireTf(base::Time::now(), base::TransformWithCovariance(tfPose));
        graph.addTransform(tf.second->parent_link_name, tf.second->child_link_name, envireTf);      
    }
}

void envire::urdf::Robot::loadLinks(envire::core::EnvireGraph &graph)
{
    if(!initialized)
    {
        return;
    }
    using linkItemPtr = envire::core::Item<::urdf::Link>::Ptr;
    for(std::pair<std::string, boost::shared_ptr<::urdf::Link> > frame : urdfModel->links_)
    {
        linkItemPtr link_itemPtr (new  envire::core::Item<::urdf::Link>(*(frame.second)));
        envire::core::FrameId frameId = frame.second->name;
        graph.addItemToFrame(frameId, link_itemPtr);      
    }
    linksLoaded = true;
}

void envire::urdf::Robot::loadJoints(envire::core::EnvireGraph& graph)
{
    if(!initialized)
    {
        return;
    }
    using jointItemPtr = envire::core::Item<::urdf::Joint>::Ptr;
    for(std::pair<std::string, boost::shared_ptr<::urdf::Joint> > tf : urdfModel->joints_) 
    {
        jointItemPtr joint_itemPtr (new envire::core::Item<::urdf::Joint>(*(tf.second)));
        envire::core::FrameId sourceId = tf.second->parent_link_name;
        graph.addItemToFrame(sourceId, joint_itemPtr);    
    }    
}

bool envire::urdf::Robot::frameHas(envire::core::EnvireGraph& graph, FRAME_ITEM_TYPE itemType, envire::core::FrameId frameID)
{
    bool hasItem=false;
    switch (itemType)
    {
        case JOINT:
        {
            envire::core::EnvireGraph::ItemIterator<envire::core::Item<::urdf::Joint*>> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::urdf::Joint*>>(frameID);
            if(begin!=end)
                hasItem=true;
            break;
        }

        case LINK:
        {
            envire::core::EnvireGraph::ItemIterator<envire::core::Item<::urdf::Link *>> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::urdf::Link *>>(frameID);
            if(begin!=end)
                hasItem=true;
            break;
        }


    }

    return hasItem;
}

std::vector<envire::core::FrameId>  envire::urdf::Robot::getTransformFrames(envire::core::FrameId &sourceFrame,envire::core::FrameId &targetFrame, envire::core::EnvireGraph &graph)
{
    return graph.getPath(sourceFrame, targetFrame);
}




