#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MotionExecutor
{
public:
    MotionExecutor() : ac_("move_base", true)
    {
        ros::NodeHandle pnh("~");
        pnh.param("min_send_interval_s", min_send_interval_s_, 4.0);
        pnh.param("goal_timeout_s", goal_timeout_s_, 40.0);
        pnh.param("improvement_threshold", improvement_threshold_, 0.0); // reserved if you add score-aware resend

        goal_sub_ = nh_.subscribe("/adaptive_goal", 10, &MotionExecutor::goalCb, this);

        ROS_INFO("Waiting for move_base server...");
        ac_.waitForServer();
        ROS_INFO("MotionExecutor ready: listening /adaptive_goal");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    MoveBaseClient ac_;

    ros::Time last_send_;
    ros::Time goal_start_;
    double min_send_interval_s_;
    double goal_timeout_s_;
    double improvement_threshold_;

    geometry_msgs::PoseStamped last_goal_;
    bool have_last_goal_ = false;

    double dist2(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) const
    {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        return dx*dx + dy*dy;
    }

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // Throttle
        if (!last_send_.isZero() && (ros::Time::now() - last_send_).toSec() < min_send_interval_s_)
            return;

        geometry_msgs::PoseStamped goal_pose = *msg;

        // Avoid resending near-identical goals
        if (have_last_goal_ && dist2(goal_pose, last_goal_) < 0.25) // 0.5m^2
            return;

        // Cancel if active
        if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ac_.cancelGoal();
            ROS_INFO("Canceled active goal (replan).");
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = goal_pose;
        goal.target_pose.header.stamp = ros::Time::now();

        ac_.sendGoal(goal);

        last_send_ = ros::Time::now();
        goal_start_ = last_send_;

        last_goal_ = goal_pose;
        have_last_goal_ = true;

        ROS_INFO("Sent adaptive goal to move_base: (%.2f, %.2f)",
                 goal_pose.pose.position.x, goal_pose.pose.position.y);
    }

public:
    void spin()
    {
        ros::Rate r(10);
        while (ros::ok())
        {
            // Timeout handling
            if (!goal_start_.isZero() &&
                ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            {
                double t = (ros::Time::now() - goal_start_).toSec();
                if (t > goal_timeout_s_)
                {
                    ROS_WARN("Goal timeout (%.1fs). Canceling.", t);
                    ac_.cancelGoal();
                    goal_start_ = ros::Time(0);
                }
            }

            ros::spinOnce();
            r.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_executor_node");
    MotionExecutor ex;
    ex.spin();
    return 0;
}