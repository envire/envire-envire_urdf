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
            
            virtual ~GraphLoader() {};

            virtual void loadStructure(const ::urdf::ModelInterface& urdfModel);
            
            virtual void loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo, const ::urdf::ModelInterface& urdfModel);
            
            virtual void loadFrames(int& nextGroupId, const ::urdf::ModelInterface& urdfModel);
                    void loadFrames(const ::urdf::ModelInterface& urdfModel);
            
            virtual void loadJoints(const ::urdf::ModelInterface& urdfModel);
            
            virtual bool setJointValue(const ::urdf::ModelInterface& urdfModel, const std::string &jointName, const float &value);
            
        private:            
            void initFrames(const ::urdf::ModelInterface& urdfModel);
            
            void initTfs(const ::urdf::ModelInterface& urdfModel);            
        };
    }
}

#endif // _ENVIRE_URDF_ROBOT_HPP
