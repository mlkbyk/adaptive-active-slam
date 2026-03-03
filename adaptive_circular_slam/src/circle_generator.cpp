#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

class CircleGenerator
{
public:
    CircleGenerator()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");

        pnh.param("points", points_, 36);
        pnh.param("base_radius", base_radius_, 1.0);
        pnh.param("frame_id", frame_id_, std::string("map"));
        pnh.param("base_frame", base_frame_, std::string("base_link"));
        pnh.param("publish_rate", publish_rate_, 1.0);
        pnh.param("enabled", enabled_, true);

        // New pipeline: no /circle_score; radius follows base_radius
        adaptive_radius_ = base_radius_;

        path_pub_ = nh_.advertise<nav_msgs::Path>("/circle_path", 10);

        ROS_INFO("CircleGenerator ready: /circle_path (enabled=%s, frame=%s base=%s)",
                 enabled_ ? "true" : "false",
                 frame_id_.c_str(),
                 base_frame_.c_str());
    }

    void spin()
    {
        ros::Rate rate(publish_rate_);

        while (ros::ok())
        {
            refreshParams();

            if (enabled_)
            {
                generateAndPublish();
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int points_ = 36;
    double base_radius_ = 1.0;
    double adaptive_radius_ = 1.0;
    double publish_rate_ = 1.0;

    bool enabled_ = true;

    std::string frame_id_ = "map";
    std::string base_frame_ = "base_link";

    void refreshParams()
    {
        ros::NodeHandle pnh("~");

        pnh.getParam("enabled", enabled_);
        pnh.getParam("base_radius", base_radius_);
        pnh.getParam("points", points_);
        pnh.getParam("publish_rate", publish_rate_);
        pnh.getParam("frame_id", frame_id_);
        pnh.getParam("base_frame", base_frame_);

        // clamp
        if (points_ < 8) points_ = 8;
        if (base_radius_ < 0.1) base_radius_ = 0.1;
        if (publish_rate_ < 0.1) publish_rate_ = 0.1;

        adaptive_radius_ = base_radius_;
    }

    void generateAndPublish()
    {
        nav_msgs::Path path;
        path.header.frame_id = frame_id_;
        path.header.stamp = ros::Time::now();
        path.poses.reserve(points_);

        geometry_msgs::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(frame_id_, base_frame_, ros::Time(0));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed (%s -> %s): %s",
                              frame_id_.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        const double cx = tf.transform.translation.x;
        const double cy = tf.transform.translation.y;

        for (int i = 0; i < points_; ++i)
        {
            const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(points_);

            geometry_msgs::PoseStamped pose;
            pose.header = path.header;

            pose.pose.position.x = cx + adaptive_radius_ * std::cos(angle);
            pose.pose.position.y = cy + adaptive_radius_ * std::sin(angle);
            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
        }

        path_pub_.publish(path);
        ROS_INFO_THROTTLE(1.0, "Published /circle_path: points=%d radius=%.2f enabled=%s",
                          points_, adaptive_radius_, enabled_ ? "true" : "false");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_generator_node");
    CircleGenerator generator;
    generator.spin();
    return 0;
}
