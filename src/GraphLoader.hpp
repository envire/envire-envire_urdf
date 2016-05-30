#ifndef _ENVIRE_URDF_ROBOT_HPP_
#define _ENVIRE_URDF_ROBOT_HPP_
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <urdf_parser/urdf_parser.h>

#include "EnvireLoader.hpp"

// TODO Documentation

namespace envire
{ 
  namespace core 
  {
      class EnvireGraph;
  }
  
    namespace urdf
    {
        class GraphLoader : public envire::core::EnvireLoader< ::urdf::ModelInterface >
        { 
        public:
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph)
                : EnvireLoader<::urdf::ModelInterface>(targetGraph) {};
            
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, envire::core::Transform pose)
                : EnvireLoader<::urdf::ModelInterface>(targetGraph, pose) {};
                
            virtual void loadStructure(boost::shared_ptr< ::urdf::ModelInterface >);
            
            virtual void loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo, boost::shared_ptr< ::urdf::ModelInterface >);
            
            virtual void loadFrames(boost::shared_ptr< ::urdf::ModelInterface >);
            
            virtual void loadJoints(boost::shared_ptr< ::urdf::ModelInterface >);
            
            
        private:            
            void initFrames(boost::shared_ptr<::urdf::ModelInterface> urdfModel);
            
            void initTfs(boost::shared_ptr<::urdf::ModelInterface> urdfModel);            
        };
    }
}

#endif // _ENVIRE_URDF_ROBOT_HPP
