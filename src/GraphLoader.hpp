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
            
            /** 
             * This method includes in the frames the visual objects that
             * the simulator will display to
             * 
             * @param urdfModel the already loaded model
             * @param filename to prefix to the mesh paths inside the visuals of the model
             */
            virtual void loadVisuals(const ::urdf::ModelInterface& urdfModel, const std::string& modelFilename = "");

            virtual bool setJointValue(const ::urdf::ModelInterface& urdfModel, const std::string &jointName, const float &value);
            
            virtual void addURIPath(const std::string& uri, const std::string& path){
                printf("set uri path urdf %s:%s\n",uri.c_str(), path.c_str());
                uriPaths[uri] = path;
            }


        private:            
            void initFrames(const ::urdf::ModelInterface& urdfModel);
            
            void initTfs(const ::urdf::ModelInterface& urdfModel);

            std::map<std::string, std::string> uriPaths;   
        };
    }
}

#endif // _ENVIRE_URDF_ROBOT_HPP
