#ifndef _ENVIRE_URDF_ROBOT_HPP_
#define _ENVIRE_URDF_ROBOT_HPP_
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <urdf_parser/urdf_parser.h>


// TODO Glossary

namespace envire
{ 
  namespace core 
  {
      class EnvireGraph;
  }
  
    namespace urdf
    {
        class Robot
        { 
        public:
            Robot(){};
            
            Robot(envire::core::Transform pose):iniPose(pose){};
            
            Robot(const std::string& path)
            {
                loadFromUrdf(path);
            };
            
            Robot(envire::core::Transform pose, const std::string& path):iniPose(pose)
            {
                loadFromUrdf(path);
            };
            
            void initGraph( envire::core::EnvireGraph& graph);
            
            void initGraph( envire::core::EnvireGraph &graph, 
                            envire::core::GraphTraits::vertex_descriptor linkTo);
            
            void loadLinks(envire::core::EnvireGraph &graph);
            
            void loadJoints(envire::core::EnvireGraph &graph);
            
//             bool frameHas(envire::core::EnvireGraph &graph,
//                           FRAME_ITEM_TYPE itemType, 
//                           envire::core::FrameId frameID);
            
//             std::vector<envire::core::FrameId> getTransformFrames(
//                 envire::core::FrameId &sourceFrame,
//                 envire::core::FrameId &targetFrame, 
//                 envire::core::EnvireGraph &graph);
            
            
        private:
            envire::core::Transform iniPose;
            boost::shared_ptr<::urdf::ModelInterface> urdfModel;
            bool initialized = false;
            const bool debug = false;
            bool linksLoaded = false;
            
            void loadFromUrdf(const std::string& path);
            
            void initFrames(envire::core::EnvireGraph &graph);
            
            void initTfs(envire::core::EnvireGraph &graph);            
        };
    }
}

#endif // _ENVIRE_URDF_ROBOT_HPP
