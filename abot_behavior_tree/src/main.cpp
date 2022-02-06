#include "movebase_client.h"
#include "interrupt_event.h"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

using namespace BT;

int main(int argc , char **argv)
{
    ros::init(argc,argv,"abot_bt");
    ros::NodeHandle nh("~");
    
    //load xml
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "/home/kamal/catkin_ws/src/abot/abot_behavior_tree/config/bt_test.xml");
    ROS_INFO("Loading XML : %s", xml_filename.c_str());

    //Use BehaviorTreeFactory to register custom nodes
    BehaviorTreeFactory factory;
    
    //Register movebase client and interrupt event nodes
    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<InterruptEvent>("InterruptEvent");
    
    //Create tree
    auto tree = factory.createTreeFromFile(xml_filename);

    //Create a logger
    StdCoutLogger logger_cout(tree);

    //Create logger to publish status changes using ZeroMQ [used by Groot]
    PublisherZMQ publisher_zmq(tree);

    //Enumerates every possible node states after execution during a particular time step.
    NodeStatus status = NodeStatus::RUNNING;

    //Keep ticking untill you get SUCCESS or FAILURE state
    while(ros::ok() && status == NodeStatus::RUNNING)
    {
        status = tree.rootNode()->executeTick();
        //Sleep 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
