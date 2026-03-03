#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <limits>
#include <cmath>

class CandidateVisualizer
{
public:
    CandidateVisualizer()
    {
        ros::NodeHandle pnh("~");
        pnh.param("frame_id", frame_id_, std::string("map"));
        pnh.param("marker_scale", marker_scale_, 0.25);

        path_sub_ = nh_.subscribe("/circle_path", 10, &CandidateVisualizer::pathCb, this);
        scores_sub_ = nh_.subscribe("/candidate_scores", 10, &CandidateVisualizer::scoresCb, this);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/best_candidate_marker", 10);

        ROS_INFO("CandidateVisualizer ready -> /best_candidate_marker");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_, scores_sub_;
    ros::Publisher marker_pub_;

    nav_msgs::Path path_;
    std_msgs::Float64MultiArray scores_;
    bool have_path_ = false;
    bool have_scores_ = false;

    std::string frame_id_;
    double marker_scale_;

    void pathCb(const nav_msgs::Path::ConstPtr& msg)
    {
        path_ = *msg;
        have_path_ = !path_.poses.empty();
        publishIfReady();
    }

    void scoresCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        scores_ = *msg;
        have_scores_ = !scores_.data.empty();
        publishIfReady();
    }

    void publishIfReady()
    {
        if (!have_path_ || !have_scores_) return;
        if (path_.poses.size() != scores_.data.size()) return;

        int best_i = -1;
        double best_s = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < scores_.data.size(); ++i)
        {
            double s = scores_.data[i];
            if (!std::isfinite(s) || s < -1e8) continue;
            if (s > best_s)
            {
                best_s = s;
                best_i = static_cast<int>(i);
            }
        }

        if (best_i < 0) return;

        const auto& pose = path_.poses[best_i].pose;

        visualization_msgs::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = ros::Time::now();
        m.ns = "adaptive_circular_slam";
        m.id = 1;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose = pose;

        m.scale.x = marker_scale_;
        m.scale.y = marker_scale_;
        m.scale.z = marker_scale_;

        // Leave default color if you want; RViz requires color alpha to be >0.
        m.color.a = 1.0;
        m.color.r = 1.0;
        m.color.g = 0.2;
        m.color.b = 0.2;

        m.lifetime = ros::Duration(0.5);

        marker_pub_.publish(m);

        ROS_INFO_THROTTLE(1.0, "Best candidate idx=%d score=%.3f", best_i, best_s);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "candidate_visualizer_node");
    CandidateVisualizer node;
    ros::spin();
    return 0;
}