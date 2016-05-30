#pragma once

#include <envire_core/graph/EnvireGraph.hpp>


namespace envire {
    namespace core {
        template <class T> class EnvireLoader
        {
        public:
            EnvireLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph): graph(targetGraph){};
            
            EnvireLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, envire::core::Transform pose): graph(targetGraph), iniPose(pose){};
            
            std::shared_ptr<envire::core::EnvireGraph> getGraph(){return this->graph;};
            
            virtual void loadStructure(const T& ) = 0;
            
            virtual void loadStructure(envire::core::GraphTraits::vertex_descriptor , const T& ) = 0;
            
            virtual void loadFrames(const T& ) = 0;
            
            virtual void loadJoints(const T& ) = 0;
            
        protected:
            std::shared_ptr<envire::core::EnvireGraph> graph;    
            envire::core::Transform iniPose;
            bool initialized = false;
            const bool debug = false;
            bool linksLoaded = false;
        };
    }
}