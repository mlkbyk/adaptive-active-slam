#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MotionExecutor
{
public:
    MotionExecutor() : ac("move_base", true)
    {
        path_sub = nh.subscribe("/circle_path", 10, &MotionExecutor::pathCallback, this);
        score_sub = nh.subscribe("/circle_score", 10, &MotionExecutor::scoreCallback, this);

        ROS_INFO("Waiting for move_base server...");
        ac.waitForServer();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    ros::Subscriber score_sub;

    MoveBaseClient ac;

    nav_msgs::Path current_path;
    bool path_received = false;
    ros::Time last_goal_time;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        current_path = *msg;
        path_received = true;
    }

    void scoreCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        if (!path_received || current_path.poses.empty())
            return;

        if ((ros::Time::now() - last_goal_time).toSec() < 5.0)
            return;

        int mid_index = current_path.poses.size() / 2;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = current_path.poses[mid_index];
        goal.target_pose.header.stamp = ros::Time::now();

        if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ac.cancelGoal();
            ROS_INFO("Canceled previous goal.");
        }

        ac.sendGoal(goal);

        ROS_INFO("New adaptive goal sent.");

        last_goal_time = ros::Time::now();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_executor_node");

    MotionExecutor executor;

    ros::spin();
    return 0;
}
