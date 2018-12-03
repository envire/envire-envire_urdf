#include "GraphLoader.hpp"
#include <iostream>
#include <urdf_model/link.h>
#include <urdf_model/types.h>
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


    void envire::urdf::GraphLoader::loadVisuals(const ::urdf::ModelInterface& urdfModel, const std::string& modelFilename)
    {
        using VisualsItemPtr = envire::core::Item<::urdf::Visual>::Ptr;
        //std::vector<::smurf::Frame *> frames= urdfModel.getFrames(); //urdf:::Link
        std::map<std::string, ::urdf::LinkSharedPtr> links = urdfModel.links_;

        //for(::smurf::Frame* frame : links)
        for(auto &link : links)
        {
            const std::vector<::urdf::VisualSharedPtr>& visuals = link.second->visual_array; //  std::vector<VisualSharedPtr>
            //NOTE used to create unique frame names for the visuals
            int visualNo = 0;
            //int groupId = frame->getGroupId();
            for(const ::urdf::VisualSharedPtr visual : visuals)
            {
                const base::Vector3d translation(visual->origin.position.x, visual->origin.position.y, visual->origin.position.z);
                const base::Quaterniond rotation(visual->origin.rotation.w, visual->origin.rotation.x, visual->origin.rotation.y, visual->origin.rotation.z);            
                VisualsItemPtr visual_itemPtr(new envire::core::Item<::urdf::Visual>(::urdf::Visual()));
                    visual_itemPtr->getData().geometry = visual->geometry;
                    visual_itemPtr->getData().material = visual->material;
                    visual_itemPtr->getData().material_name = visual->material_name;
                    visual_itemPtr->getData().name = visual->name;
                    visual_itemPtr->getData().origin = visual->origin;
                
                //prefix local mesh paths with the urdf path
                if (visual->geometry->type == ::urdf::Geometry::MESH){
                    ::urdf::Mesh* mesh = dynamic_cast<::urdf::Mesh*>(visual->geometry.get());
                    //remove all afer last "/" from modelFilename
                    size_t found = modelFilename.find_last_of("/\\");
                    mesh->filename = modelFilename.substr(0,found) + "/" +  mesh->filename;
                }
 
                //visual_itemPtr->getData().groupId = groupId;
                //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                if(translation == base::Vector3d::Zero() && 
                    (rotation.coeffs() == base::Quaterniond::Identity().coeffs() ||
                    rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                {
                    graph->addItemToFrame(link.first, visual_itemPtr);
                }
                else
                {
                    envire::core::Transform tf(translation, rotation);
                    const envire::core::FrameId visualFrame(link.first + "_visual_" + boost::lexical_cast<envire::core::FrameId>(visualNo) );
                    ++visualNo;
                    graph->addTransform(link.first, visualFrame, tf);
                    graph->addItemToFrame(visualFrame, visual_itemPtr);
                }
            }
            //if (debug) LOG_DEBUG("[GraphLoader::loadVisuals] Added smurf::Visuals" );
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

        //create rpy quat
        Eigen::Quaterniond origin;
        double x,y,z,w;
        joint->parent_to_joint_origin_transform.rotation.getQuaternion(x,y,z,w);
        origin = Eigen::Quaterniond(w,x,y,z);

        switch(joint->type){
            case ::urdf::Joint::REVOLUTE:
            case ::urdf::Joint::CONTINUOUS:
            {
                Eigen::Vector3d axis (joint->axis.x,joint->axis.y,joint->axis.z);
                Eigen::AngleAxisd angleaxis (value,axis);

                //get envire joint
                envire::core::Transform tf = graph->getTransform(joint->parent_link_name,joint->child_link_name);
                tf.transform.orientation = origin * Eigen::Quaterniond(angleaxis);
                graph->updateTransform(joint->parent_link_name,joint->child_link_name,tf);

                return true;
            }
            case ::urdf::Joint::UNKNOWN:
            case ::urdf::Joint::PRISMATIC:
            case ::urdf::Joint::FLOATING:
            case ::urdf::Joint::PLANAR:
            case ::urdf::Joint::FIXED:
                printf("Joint type not supported for setting values of %s\n",jointName.c_str());
                return false;
        }
    }
