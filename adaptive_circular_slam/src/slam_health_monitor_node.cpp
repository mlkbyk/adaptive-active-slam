#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <algorithm>

class SlamHealthMonitor
{
public:
    SlamHealthMonitor()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");

        pnh.param("map_frame", map_frame_, std::string("map"));
        pnh.param("base_frame", base_frame_, std::string("base_link"));

        pnh.param("cov_k_gain", cov_k_gain_, 5.0);              // exp(-k*uncertainty)
        pnh.param("tf_jump_threshold_m", tf_jump_threshold_m_, 0.35);
        pnh.param("tf_jump_penalty", tf_jump_penalty_, 0.35);   // subtract if jump occurs

        pnh.param("publish_rate", publish_rate_, 10.0);
        pnh.param("ema_alpha", ema_alpha_, 0.15);               // smoothing on health

        amcl_sub_ = nh_.subscribe("/amcl_pose", 10, &SlamHealthMonitor::amclCb, this);
        health_pub_ = nh_.advertise<std_msgs::Float64>("/slam_health", 10);

        last_tf_time_ = ros::Time(0);
        have_tf_pose_ = false;

        ROS_INFO("SlamHealthMonitor ready -> publishing /slam_health");
    }

    void spin()
    {
        ros::Rate r(publish_rate_);
        while (ros::ok())
        {
            updateTfHealth();
            publishHealth();

            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_sub_;
    ros::Publisher health_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string map_frame_;
    std::string base_frame_;

    double cov_k_gain_;
    double tf_jump_threshold_m_;
    double tf_jump_penalty_;
    double publish_rate_;
    double ema_alpha_;

    // AMCL-based confidence
    bool have_amcl_ = false;
    double amcl_score_ = 1.0; // 0..1

    // TF jump detection
    bool have_tf_pose_;
    tf2::Vector3 last_tf_pos_;
    ros::Time last_tf_time_;
    double tf_score_ = 1.0;   // 0..1

    // Final health (smoothed)
    bool have_health_ = false;
    double health_ema_ = 1.0;

    static double clamp01(double v)
    {
        return std::max(0.0, std::min(1.0, v));
    }

    void amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // Use XY covariance as uncertainty proxy
        double var_x = msg->pose.covariance[0];
        double var_y = msg->pose.covariance[7];

        double uncertainty = std::sqrt(std::max(0.0, var_x) + std::max(0.0, var_y));
        double score = std::exp(-cov_k_gain_ * uncertainty); // 0..1

        amcl_score_ = clamp01(score);
        have_amcl_ = true;
    }

    void updateTfHealth()
    {
        geometry_msgs::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex.what());
            return;
        }

        tf2::Vector3 pos(tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z);

        if (!have_tf_pose_)
        {
            last_tf_pos_ = pos;
            have_tf_pose_ = true;
            last_tf_time_ = ros::Time::now();
            tf_score_ = 1.0;
            return;
        }

        double jump = (pos - last_tf_pos_).length();
        last_tf_pos_ = pos;
        last_tf_time_ = ros::Time::now();

        // If we detect a sudden jump, penalize tf_score briefly
        if (jump > tf_jump_threshold_m_)
        {
            tf_score_ = clamp01(tf_score_ - tf_jump_penalty_);
            ROS_WARN("TF jump detected: %.3fm -> tf_score=%.2f", jump, tf_score_);
        }
        else
        {
            // recover slowly
            tf_score_ = clamp01(tf_score_ + 0.02);
        }
    }

    void publishHealth()
    {
        // If AMCL not available, assume neutral (1.0)
        double amcl = have_amcl_ ? amcl_score_ : 1.0;
        double tfh  = tf_score_;

        // Combine (multiplicative is strict & realistic)
        double raw = clamp01(amcl * tfh);

        // Smooth with EMA to avoid flicker
        if (!have_health_)
        {
            health_ema_ = raw;
            have_health_ = true;
        }
        else
        {
            health_ema_ = (1.0 - ema_alpha_) * health_ema_ + ema_alpha_ * raw;
        }

        std_msgs::Float64 msg;
        msg.data = clamp01(health_ema_);
        health_pub_.publish(msg);

        ROS_INFO_THROTTLE(1.0, "Health=%.2f (amcl=%.2f tf=%.2f)", msg.data, amcl, tfh);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_health_monitor_node");
    SlamHealthMonitor node;
    node.spin();
    return 0;
}