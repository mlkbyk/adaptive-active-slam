#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <sstream>
#include <iomanip>

class ACSControlPanel
{
public:
    ACSControlPanel()
    {
        ros::NodeHandle pnh("~");

        pnh.param("circle_ns", circle_ns_, std::string("/acs_circle_generator"));
        pnh.param("gain_ns", gain_ns_, std::string("/acs_gain_evaluator"));

        health_sub_ = nh_.subscribe("/slam_health", 10, &ACSControlPanel::healthCb, this);
        score_sub_  = nh_.subscribe("/acs/best_score", 10, &ACSControlPanel::scoreCb, this);
        mode_sub_   = nh_.subscribe("/acs/mode", 10, &ACSControlPanel::modeCb, this);

        dash_pub_ = nh_.advertise<std_msgs::String>("/acs/dashboard_text", 10);

        start_srv_ = nh_.advertiseService("/acs/start", &ACSControlPanel::startSrv, this);
        stop_srv_  = nh_.advertiseService("/acs/stop", &ACSControlPanel::stopSrv, this);
        radius_srv_ = nh_.advertiseService("/acs/set_radius", &ACSControlPanel::radiusSrv, this);
        eval_radius_srv_ = nh_.advertiseService("/acs/set_eval_radius", &ACSControlPanel::evalRadiusSrv, this);

        ROS_INFO("ACS Control Panel ready.");
    }

    void spin()
    {
        ros::Rate r(5);
        while (ros::ok())
        {
            publishDashboard();
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber health_sub_, score_sub_, mode_sub_;
    ros::Publisher dash_pub_;

    ros::ServiceServer start_srv_, stop_srv_;
    ros::ServiceServer radius_srv_, eval_radius_srv_;

    std::string circle_ns_;
    std::string gain_ns_;

    double health_ = 1.0;
    double best_score_ = 0.0;
    std::string mode_ = "PAUSE";

    void healthCb(const std_msgs::Float64::ConstPtr& m){ health_ = m->data; }
    void scoreCb(const std_msgs::Float64::ConstPtr& m){ best_score_ = m->data; }
    void modeCb(const std_msgs::String::ConstPtr& m){ mode_ = m->data; }

    bool startSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        ros::param::set(circle_ns_ + "/enabled", true);
        res.success = true;
        res.message = "ACS started";
        return true;
    }

    bool stopSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        ros::param::set(circle_ns_ + "/enabled", false);
        res.success = true;
        res.message = "ACS stopped";
        return true;
    }

    bool radiusSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        double r = req.data ? 2.0 : 1.0;  // quick toggle
        ros::param::set(circle_ns_ + "/base_radius", r);
        res.success = true;
        res.message = "Radius changed";
        return true;
    }

    bool evalRadiusSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        double r = req.data ? 3.0 : 2.0;
        ros::param::set(gain_ns_ + "/eval_radius_m", r);
        res.success = true;
        res.message = "Eval radius changed";
        return true;
    }

    void publishDashboard()
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "\n===== ACS CONTROL PANEL =====\n"
           << "Health:      " << health_ << "\n"
           << "Best Score:  " << best_score_ << "\n"
           << "Mode:        " << mode_ << "\n"
           << "=============================";

        std_msgs::String msg;
        msg.data = ss.str();
        dash_pub_.publish(msg);

        ROS_INFO_THROTTLE(1.0, "%s", msg.data.c_str());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "acs_control_panel_node");
    ACSControlPanel node;
    node.spin();
    return 0;
}