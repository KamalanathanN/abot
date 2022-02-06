#pragma once
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>

// Custom type
struct Pose2D
{
    double x, y, quaternion_z, quaternion_w;
};

namespace BT
{
    template <>inline
    Pose2D convertFromString(StringView key)
    {
        // four number seperated by semicolons
        auto pose = BT::splitString(key,';');
        if(pose.size() != 4)
            throw BT::RuntimeError("Invalid input !");
        else
        {
            Pose2D output;
            output.x = convertFromString<double>(pose[0]);
            output.y = convertFromString<double>(pose[1]);
            output.quaternion_z = convertFromString<double>(pose[2]);
            output.quaternion_w = convertFromString<double>(pose[3]);
            return output;
        }
    }
}

class MoveBase : public BT::AsyncActionNode
{
    public:
        MoveBase(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config), _client("move_base", true)
            {}
        
        // Defining static method is mandatory
        static BT::PortsList providedPorts()
        {
            return{BT::InputPort<Pose2D>("goal")};
        }
        
        virtual BT::NodeStatus tick() override;

        virtual void halt() override
        {
            _aborted = true;
        }
    private:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        MoveBaseClient _client;
        bool _aborted;
};
