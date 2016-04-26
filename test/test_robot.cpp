#include <boost/test/unit_test.hpp>
#include <envire_urdf/Robot.hpp>

const std::string path="./urdf/two_boxes.urdf";

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::urdf::Robot robot;
    envire::urdf::Robot robotPath(path);
}

