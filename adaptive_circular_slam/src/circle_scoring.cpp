#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

class CircleScoring
{
public:
    CircleScoring()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("k_gain", k_gain, 5.0);

        score_pub = nh.advertise<std_msgs::Float64>("/circle_score", 10);
        amcl_sub = nh.subscribe("/amcl_pose", 10, &CircleScoring::amclCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher score_pub;
    ros::Subscriber amcl_sub;

    double k_gain;

    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        double var_x = msg->pose.covariance[0];
        double var_y = msg->pose.covariance[7];

        double uncertainty = std::sqrt(var_x + var_y);

        // Convert uncertainty to confidence score (0-1)
        double score = std::exp(-k_gain * uncertainty);

        std_msgs::Float64 score_msg;
        score_msg.data = score;

        ROS_INFO("Uncertainty: %.4f -> Score: %.3f", uncertainty, score);

        score_pub.publish(score_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_scoring_node");

    CircleScoring scoring;

    ros::spin();
    return 0;
}
