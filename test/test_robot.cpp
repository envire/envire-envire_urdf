#include <boost/test/unit_test.hpp>
#include <envire_urdf/Robot.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>

const std::string path="../../test/urdf/two_boxes.urdf";
const std::string pathDynamic="../../test/urdf/two_boxes_dynamic_joint.urdf";

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::urdf::Robot robot;
    envire::urdf::Robot robotPath(path);
}

BOOST_AUTO_TEST_CASE(initRobotGraph_noPos)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::urdf::Robot robot(path);
    robot.initGraph(transformGraph);
    viz.write(transformGraph, "initRobotGraph_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withPos)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withDynamicJoint)
{   
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << pathDynamic << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::Robot robot(iniPose, pathDynamic);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withDynamicJoint_test.dot");
}

envire::urdf::Robot getRobotWithInitGraph(const std::string path, envire::core::EnvireGraph& transformGraph)
{

    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::urdf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    return robot;
}

BOOST_AUTO_TEST_CASE(loadLinks)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::urdf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    robot.loadLinks(transformGraph);
    viz.write(transformGraph, "loadLinks_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadLinks_withDynamicJoint)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::urdf::Robot robot = getRobotWithInitGraph(pathDynamic, transformGraph);
    robot.loadLinks(transformGraph);
    viz.write(transformGraph, "loadLinks_withDynamicJoint_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadJoints)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::urdf::Robot robot = getRobotWithInitGraph(pathDynamic, transformGraph);
    robot.loadJoints(transformGraph);
    viz.write(transformGraph, "loadJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(completeTest)
{
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::urdf::Robot robot = getRobotWithInitGraph(pathDynamic, transformGraph);
    robot.loadLinks(transformGraph);
    robot.loadJoints(transformGraph);
    viz.write(transformGraph, "completeTest_Test.dot");
}