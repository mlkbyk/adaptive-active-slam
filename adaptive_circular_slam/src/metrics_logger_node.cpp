#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <fstream>
#include <string>
#include <iomanip>

class MetricsLogger
{
public:
    MetricsLogger()
    {
        ros::NodeHandle pnh("~");
        pnh.param("log_path", log_path_, std::string("/tmp/acs_metrics.csv"));

        health_sub_ = nh_.subscribe("/slam_health", 10, &MetricsLogger::healthCb, this);
        best_score_sub_ = nh_.subscribe("/acs/best_score", 10, &MetricsLogger::bestScoreCb, this);
        idx_sub_ = nh_.subscribe("/acs/selected_idx", 10, &MetricsLogger::idxCb, this);
        mode_sub_ = nh_.subscribe("/acs/mode", 10, &MetricsLogger::modeCb, this);
        goal_sub_ = nh_.subscribe("/adaptive_goal", 10, &MetricsLogger::goalCb, this);

        file_.open(log_path_, std::ios::out);
        if (!file_.is_open())
        {
            ROS_ERROR("Failed to open log file: %s", log_path_.c_str());
            return;
        }

        file_ << "t,health,best_score,selected_idx,mode,goal_x,goal_y\n";
        file_.flush();

        ROS_INFO("MetricsLogger writing to %s", log_path_.c_str());
    }

    ~MetricsLogger()
    {
        if (file_.is_open()) file_.close();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber health_sub_, best_score_sub_, idx_sub_, mode_sub_, goal_sub_;

    std::string log_path_;
    std::ofstream file_;

    double health_ = 1.0;
    double best_score_ = -1e9;
    int selected_idx_ = -1;
    std::string mode_ = "PAUSE";
    double goal_x_ = 0.0, goal_y_ = 0.0;
    bool have_goal_ = false;

    void healthCb(const std_msgs::Float64::ConstPtr& m) { health_ = m->data; writeRow(); }
    void bestScoreCb(const std_msgs::Float64::ConstPtr& m) { best_score_ = m->data; writeRow(); }
    void idxCb(const std_msgs::Int32::ConstPtr& m) { selected_idx_ = m->data; writeRow(); }
    void modeCb(const std_msgs::String::ConstPtr& m) { mode_ = m->data; writeRow(); }

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& m)
    {
        goal_x_ = m->pose.position.x;
        goal_y_ = m->pose.position.y;
        have_goal_ = true;
        writeRow();
    }

    void writeRow()
    {
        if (!file_.is_open()) return;

        const double t = ros::Time::now().toSec();

        file_ << std::fixed << std::setprecision(6)
              << t << ","
              << health_ << ","
              << best_score_ << ","
              << selected_idx_ << ","
              << mode_ << ","
              << (have_goal_ ? goal_x_ : 0.0) << ","
              << (have_goal_ ? goal_y_ : 0.0)
              << "\n";

        file_.flush();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "metrics_logger_node");
    MetricsLogger n;
    ros::spin();
    return 0;
}