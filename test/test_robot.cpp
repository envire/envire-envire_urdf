#include <boost/test/unit_test.hpp>
#include <envire_urdf/GraphLoader.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>

const std::string path="../../test/urdf/two_boxes.urdf";
const std::string pathDynamic="../../test/urdf/two_boxes_dynamic_joint.urdf";

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    std::shared_ptr<envire::core::EnvireGraph> transformGraph;
    envire::urdf::GraphLoader loader(transformGraph);
    envire::core::Transform iniPose;
    envire::urdf::GraphLoader robotPath(transformGraph,iniPose);
}

BOOST_AUTO_TEST_CASE(initRobotGraph_noPos)
{
    std::shared_ptr<envire::core::EnvireGraph>  transformGraph( new envire::core::EnvireGraph());
    urdf::ModelInterface robot = *urdf::parseURDFFile(path).get();
    envire::core::GraphViz viz;
    envire::urdf::GraphLoader loader(transformGraph);
    loader.loadStructure(robot);
    viz.write(*transformGraph, "initRobotGraph_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withPos)
{
    std::shared_ptr<envire::core::EnvireGraph>  transformGraph( new envire::core::EnvireGraph());
    urdf::ModelInterface robot = *urdf::parseURDFFile(path).get();
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::GraphLoader loader(transformGraph,iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    loader.loadStructure(transformGraph->getVertex(center), robot);
    viz.write(*transformGraph, "initRobotGraph_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withDynamicJoint)
{   
    std::shared_ptr<envire::core::EnvireGraph>  transformGraph( new envire::core::EnvireGraph());
    urdf::ModelInterface robot = *urdf::parseURDFFile(pathDynamic).get();
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << pathDynamic << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::GraphLoader loader(transformGraph,iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    loader.loadStructure(transformGraph->getVertex(center), robot);
    viz.write(*transformGraph, "initRobotGraph_withDynamicJoint_test.dot");
}

envire::urdf::GraphLoader getRobotWithInitGraph(urdf::ModelInterface& robot)
{
    std::shared_ptr<envire::core::EnvireGraph> transformGraph( new envire::core::EnvireGraph());
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::GraphLoader loader(transformGraph,iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    loader.loadStructure(transformGraph->getVertex(center), robot);
    return loader;
}

BOOST_AUTO_TEST_CASE(loadFrames)
{
    
    urdf::ModelInterface robot = *urdf::parseURDFFile(pathDynamic);
    envire::core::GraphViz viz;
    envire::urdf::GraphLoader loader = getRobotWithInitGraph(robot);
    loader.loadFrames(robot);
    viz.write(*loader.getGraph(), "loadFrames_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFrames_withDynamicJoint)
{
    urdf::ModelInterface robot = *urdf::parseURDFFile(pathDynamic);
    envire::core::GraphViz viz;
    envire::urdf::GraphLoader loader = getRobotWithInitGraph(robot);
    loader.loadFrames(robot);
    viz.write(*loader.getGraph(), "loadFrames_withDynamicJoint_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadJoints)
{
    urdf::ModelInterface robot = *urdf::parseURDFFile(pathDynamic);
    envire::core::GraphViz viz;
    envire::urdf::GraphLoader loader = getRobotWithInitGraph(robot);
    loader.loadJoints(robot);
    viz.write(*loader.getGraph(), "loadJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(completeTest)
{
    urdf::ModelInterface robot = *urdf::parseURDFFile(pathDynamic);
    envire::core::GraphViz viz;
    envire::urdf::GraphLoader loader = getRobotWithInitGraph(robot);
    loader.loadFrames(robot);
    loader.loadJoints(robot);
    viz.write(*loader.getGraph(), "completeTest_Test.dot");
}