#include "GraphLoader.hpp"
#include <iostream>
#include <urdf_model/link.h>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

void envire::urdf::GraphLoader::loadStructure(const ::urdf::ModelInterface& urdfModel)
{
    initialized = true;
    initFrames(urdfModel);
    initTfs(urdfModel);
    iniPose = envire::core::Transform(base::Position(0, 0, 0), Eigen::Quaterniond::Identity());
}

void envire::urdf::GraphLoader::loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo, const ::urdf::ModelInterface& urdfModel)
{
    if (!initialized)
    {
        loadStructure(urdfModel);
    }
    envire::core::FrameId robotRoot = urdfModel.getRoot()->name;
    iniPose.time = base::Time::now();
    graph->addTransform(graph->getFrameId(linkTo), robotRoot, iniPose);
}

void envire::urdf::GraphLoader::initFrames(const ::urdf::ModelInterface& urdfModel)
{
    for(std::pair<std::string, ::urdf::LinkSharedPtr > frame : urdfModel.links_)
    {
       graph->addFrame(frame.second->name);
    }

}

void envire::urdf::GraphLoader::initTfs(const ::urdf::ModelInterface& urdfModel)
{
    for(std::pair<std::string, ::urdf::JointSharedPtr > tf : urdfModel.joints_) 
    {
        Eigen::Affine3d tfPose(Eigen::Affine3d::Identity());
        ::urdf::Vector3 pos = tf.second->parent_to_joint_origin_transform.position;
        ::urdf::Rotation rot = tf.second->parent_to_joint_origin_transform.rotation;
        tfPose.linear() = Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z).matrix();
        tfPose.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);
        envire::core::Transform envireTf(base::Time::now(), base::TransformWithCovariance(tfPose));
        graph->addTransform(tf.second->parent_link_name, tf.second->child_link_name, envireTf);
    }
}

void envire::urdf::GraphLoader::loadFrames(int& nextGroupId, const ::urdf::ModelInterface& urdfModel)
{
    loadFrames(urdfModel);
}

void envire::urdf::GraphLoader::loadFrames(const ::urdf::ModelInterface& urdfModel)
{
    if(!initialized)
    {
        return;
    }
    using linkItemPtr = envire::core::Item<::urdf::Link>::Ptr;
    for(std::pair<std::string, ::urdf::LinkSharedPtr > frame : urdfModel.links_)
    {
        linkItemPtr link_itemPtr (new  envire::core::Item<::urdf::Link>(*(frame.second)));
        envire::core::FrameId frameId = frame.second->name;
        graph->addItemToFrame(frameId, link_itemPtr);
    }
    framesLoaded = true;
}

void envire::urdf::GraphLoader::loadJoints(const ::urdf::ModelInterface& urdfModel)
{
    if(!initialized)
    {
        return;
    }
    using jointItemPtr = envire::core::Item<::urdf::Joint>::Ptr;
    for(std::pair<std::string, ::urdf::JointSharedPtr > tf : urdfModel.joints_)
    {
        jointItemPtr joint_itemPtr (new envire::core::Item<::urdf::Joint>(*(tf.second)));
        envire::core::FrameId sourceId = tf.second->parent_link_name;
        graph->addItemToFrame(sourceId, joint_itemPtr);
    }
}





    bool envire::urdf::GraphLoader::setJointValue(const ::urdf::ModelInterface& urdfModel, const std::string &jointName, const float &value)
    {
        
        std::map<std::string, ::urdf::JointSharedPtr>::const_iterator joint_it = urdfModel.joints_.find(jointName);
        if (joint_it == urdfModel.joints_.end()){
            printf("Joint %s not found in model\n",jointName.c_str());
            return false;
        }
        ::urdf::JointConstSharedPtr joint = joint_it->second;
        switch(joint->type){
            case ::urdf::Joint::REVOLUTE:{
                Eigen::Vector3d axis (joint->axis.x,joint->axis.y,joint->axis.z);
                Eigen::AngleAxisd angleaxis (value,axis);

                //get envire joint
                envire::core::Transform tf = graph->getTransform(joint->parent_link_name,joint->child_link_name);
                tf.transform.orientation = Eigen::Quaterniond(angleaxis);
                graph->updateTransform(joint->parent_link_name,joint->child_link_name,tf);

                return true;
                }
            case ::urdf::Joint::UNKNOWN:
            case ::urdf::Joint::CONTINUOUS:
            case ::urdf::Joint::PRISMATIC:
            case ::urdf::Joint::FLOATING:
            case ::urdf::Joint::PLANAR:
            case ::urdf::Joint::FIXED: printf("Joint type not supported for setting values\n",jointName.c_str()); return false; break;
        }

            // Eigen::Affine3d tfPose(Eigen::Affine3d::Identity());
            // ::urdf::Vector3 pos = tf.second->parent_to_joint_origin_transform.position;
            // ::urdf::Rotation rot = tf.second->parent_to_joint_origin_transform.rotation;
            // tfPose.linear() = Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z).matrix();
            // tfPose.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);
            // envire::core::Transform envireTf(base::Time::now(), base::TransformWithCovariance(tfPose));
            // graph->addTransform(tf.second->parent_link_name, tf.second->child_link_name, envireTf);
    }
