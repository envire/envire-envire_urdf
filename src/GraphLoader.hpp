#ifndef _ENVIRE_URDF_ROBOT_HPP_
#define _ENVIRE_URDF_ROBOT_HPP_
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <urdf_parser/urdf_parser.h>


// TODO Documentation

namespace envire
{ 
  namespace core 
  {
      class EnvireGraph;
  }
  
    namespace urdf
    {
        class GraphLoader
        { 
        public:
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph): graph(targetGraph){};
            
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, envire::core::Transform pose): graph(targetGraph), iniPose(pose){};
            
            std::shared_ptr<envire::core::EnvireGraph> getGraph(){return this->graph;};
            
            void loadStructure(boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            void loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo, boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            void loadFrames(boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            void loadJoints(boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            
        private:
            std::shared_ptr<envire::core::EnvireGraph> graph;
            envire::core::Transform iniPose;
            bool initialized = false;
            const bool debug = false;
            bool linksLoaded = false;
            
            void initFrames(boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            void initTfs(boost::shared_ptr<::urdf::ModelInterface> urdfModel);            
        };
    }
}

#endif // _ENVIRE_URDF_ROBOT_HPP
