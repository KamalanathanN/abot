#include "movebase_client.h"

BT::NodeStatus MoveBase::tick()
{
    // if no server found, fail after 2s
    if(!_client.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("move_base server not found");
        return BT::NodeStatus::FAILURE;
    }

    // Get goal from InputPort of Node
    Pose2D goal;
    if(!getInput<Pose2D>("goal",goal))
    {
        // not getting input is due to wrong BT config,so throw exception instead of returning FAILURE
        throw BT::RuntimeError("missing required input [goal]");
    }

    // Reset flag
    _aborted = false;

    ROS_INFO("Sending goal %f: %f: %f: %f: ", goal.x, goal.y, goal.quaternion_z, goal.quaternion_w);

    // Build message from Pose2D
    move_base_msgs::MoveBaseGoal msg;
    msg.target_pose.header.frame_id = "map";
    msg.target_pose.header.stamp = ros::Time::now();
    msg.target_pose.pose.position.x = goal.x;
    msg.target_pose.pose.position.y = goal.y;
    msg.target_pose.pose.orientation.z = goal.quaternion_z;
    msg.target_pose.pose.orientation.w = goal.quaternion_w;

    // Send goal
    _client.sendGoal(msg);

    while (!_aborted && !_client.waitForResult(ros::Duration(0.02)))
    {
        //polling at 50hz.
    }
    
    if(_aborted)
    {
        // happens when halt() is invoked
        ROS_ERROR("MoveBase Aborted!");
        return BT::NodeStatus::FAILURE;
    }

    if(_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("MoveBase Failed");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("Target Reached !");
    return BT::NodeStatus::SUCCESS;
}