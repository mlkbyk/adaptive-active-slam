#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <limits>
#include <cmath>
#include <algorithm>
#include <string>

class DecisionManager
{
public:
    DecisionManager()
    {
        ros::NodeHandle pnh("~");

        pnh.param("publish_rate", publish_rate_, 5.0);
        pnh.param("min_replan_interval_s", min_replan_interval_s_, 3.0);

        pnh.param("hysteresis_margin", hysteresis_margin_, 0.08);
        pnh.param("min_goal_separation_m", min_goal_sep_m_, 0.5);

        // Mode thresholds
        pnh.param("health_pause_threshold", health_pause_th_, 0.25); // below -> PAUSE (no goal)
        pnh.param("health_recover_threshold", health_recover_th_, 0.45); // below -> RECOVER (conservative)
        pnh.param("health_explore_threshold", health_explore_th_, 0.70); // above -> EXPLORE (aggressive)

        // Optional conservative distance penalty
        pnh.param("use_distance_penalty", use_distance_penalty_, true);
        pnh.param("w_distance", w_distance_, 0.15);

        // If robot pose isn't available yet, keep 0,0 (later you can TF this)
        pnh.param("robot_x", robot_x_, 0.0);
        pnh.param("robot_y", robot_y_, 0.0);

        path_sub_   = nh_.subscribe("/circle_path", 10, &DecisionManager::pathCb, this);
        scores_sub_ = nh_.subscribe("/candidate_scores", 10, &DecisionManager::scoresCb, this);
        health_sub_ = nh_.subscribe("/slam_health", 10, &DecisionManager::healthCb, this);

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/adaptive_goal", 10);

        // New “professional” debug/metrics topics
        best_score_pub_  = nh_.advertise<std_msgs::Float64>("/acs/best_score", 10);
        selected_idx_pub_= nh_.advertise<std_msgs::Int32>("/acs/selected_idx", 10);
        mode_pub_        = nh_.advertise<std_msgs::String>("/acs/mode", 10);

        ROS_INFO("DecisionManager ready -> /adaptive_goal + /acs/* topics");
    }

    void spin()
    {
        ros::Rate r(publish_rate_);
        while (ros::ok())
        {
            tick();
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_, scores_sub_, health_sub_;

    ros::Publisher goal_pub_;
    ros::Publisher best_score_pub_;
    ros::Publisher selected_idx_pub_;
    ros::Publisher mode_pub_;

    nav_msgs::Path path_;
    bool have_path_ = false;

    std_msgs::Float64MultiArray scores_;
    bool have_scores_ = false;

    double health_ = 1.0;

    double publish_rate_;
    double min_replan_interval_s_;
    double hysteresis_margin_;
    double min_goal_sep_m_;

    double health_pause_th_;
    double health_recover_th_;
    double health_explore_th_;

    bool use_distance_penalty_;
    double w_distance_;
    double robot_x_, robot_y_;

    ros::Time last_publish_;

    bool have_selected_ = false;
    int selected_idx_ = -1;
    double selected_score_ = -std::numeric_limits<double>::infinity();
    geometry_msgs::PoseStamped last_goal_;

    static double clamp01(double v)
    {
        return std::max(0.0, std::min(1.0, v));
    }

    void pathCb(const nav_msgs::Path::ConstPtr& msg)
    {
        path_ = *msg;
        have_path_ = !path_.poses.empty();
    }

    void scoresCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        scores_ = *msg;
        have_scores_ = !scores_.data.empty();
    }

    void healthCb(const std_msgs::Float64::ConstPtr& msg)
    {
        health_ = clamp01(msg->data);
    }

    bool shouldReplanNow() const
    {
        if (last_publish_.isZero()) return true;
        return (ros::Time::now() - last_publish_).toSec() >= min_replan_interval_s_;
    }

    double dist2ToRobot(const geometry_msgs::PoseStamped& p) const
    {
        double dx = p.pose.position.x - robot_x_;
        double dy = p.pose.position.y - robot_y_;
        return dx*dx + dy*dy;
    }

    double dist2(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) const
    {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        return dx*dx + dy*dy;
    }

    std::string computeMode() const
    {
        if (!have_path_ || !have_scores_) return "PAUSE";
        if (health_ < health_pause_th_) return "PAUSE";
        if (health_ < health_recover_th_) return "RECOVER";
        if (health_ > health_explore_th_) return "EXPLORE";
        return "NORMAL";
    }

    void publishDebug(int best_i, double best_val, const std::string& mode)
    {
        std_msgs::Float64 bs; bs.data = best_val;
        best_score_pub_.publish(bs);

        std_msgs::Int32 si; si.data = best_i;
        selected_idx_pub_.publish(si);

        std_msgs::String m; m.data = mode;
        mode_pub_.publish(m);
    }

    void tick()
    {
        if (!have_path_ || !have_scores_) return;
        if (path_.poses.size() != scores_.data.size())
        {
            ROS_WARN_THROTTLE(2.0, "Path size (%zu) != scores size (%zu).",
                              path_.poses.size(), scores_.data.size());
            return;
        }

        const std::string mode = computeMode();

        // Safety gate: PAUSE means do not send goals
        if (mode == "PAUSE")
        {
            // Still publish debug so UI/plots work
            publishDebug(-1, -1e9, mode);
            ROS_WARN_THROTTLE(1.0, "Mode=PAUSE (health=%.2f). Not publishing goals.", health_);
            return;
        }

        if (!shouldReplanNow()) return;

        bool conservative = (mode == "RECOVER");
        bool aggressive   = (mode == "EXPLORE");

        double hysteresis = hysteresis_margin_;
        if (conservative) hysteresis *= 1.8;
        if (aggressive)   hysteresis *= 0.9;

        int best_i = -1;
        double best_val = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < scores_.data.size(); ++i)
        {
            double s = scores_.data[i];
            if (!std::isfinite(s) || s < -1e8) continue;

            if (use_distance_penalty_ && conservative)
            {
                // penalize far goals in RECOVER
                double d = std::sqrt(std::max(0.0, dist2ToRobot(path_.poses[i])));
                s -= w_distance_ * d;
            }

            if (s > best_val)
            {
                best_val = s;
                best_i = static_cast<int>(i);
            }
        }

        // Publish debug even if we didn't find any valid candidate
        publishDebug(best_i, best_val, mode);

        if (best_i < 0) return;

        geometry_msgs::PoseStamped goal = path_.poses[best_i];
        goal.header.stamp = ros::Time::now();

        // Hysteresis: don’t switch unless significantly better
        if (have_selected_)
        {
            if (best_val < selected_score_ + hysteresis)
                return;

            // Avoid micro moves
            if (dist2(goal, last_goal_) < (min_goal_sep_m_ * min_goal_sep_m_))
                return;
        }

        selected_idx_ = best_i;
        selected_score_ = best_val;
        last_goal_ = goal;
        have_selected_ = true;

        goal_pub_.publish(goal);
        last_publish_ = ros::Time::now();

        ROS_INFO("Decision: idx=%d score=%.3f health=%.2f mode=%s",
                 selected_idx_, selected_score_, health_, mode.c_str());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_manager_node");
    DecisionManager node;
    node.spin();
    return 0;
}