#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

class GainEvaluator
{
public:
    GainEvaluator()
    {
        ros::NodeHandle pnh("~");

        pnh.param("eval_radius_m", eval_radius_m_, 2.5);
        pnh.param("unknown_value", unknown_value_, -1);
        pnh.param("occupied_threshold", occupied_threshold_, 65);

        // scoring weights
        pnh.param("w_gain", w_gain_, 1.0);
        pnh.param("w_risk", w_risk_, 1.0);
        pnh.param("w_health", w_health_, 0.5);

        // optional: encourage exploration near unknown boundaries a bit
        pnh.param("boundary_bonus", boundary_bonus_, 0.10);

        // subscriptions
        map_sub_  = nh_.subscribe("/map", 1, &GainEvaluator::mapCb, this);
        path_sub_ = nh_.subscribe("/circle_path", 1, &GainEvaluator::pathCb, this);
        health_sub_ = nh_.subscribe("/slam_health", 10, &GainEvaluator::healthCb, this);

        // publish only scores (decision_manager will choose goal)
        scores_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/candidate_scores", 10);

        ROS_INFO("GainEvaluator ready: /map + /circle_path -> /candidate_scores");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_, path_sub_, health_sub_;
    ros::Publisher scores_pub_;

    nav_msgs::OccupancyGrid map_;
    bool have_map_ = false;

    nav_msgs::Path path_;
    bool have_path_ = false;

    double health_ = 1.0; // 0..1

    double eval_radius_m_;
    int unknown_value_;
    int occupied_threshold_;

    double w_gain_, w_risk_, w_health_;
    double boundary_bonus_;

    static double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;

        const size_t expected = static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height);
        if (map_.data.size() != expected)
        {
            ROS_WARN_THROTTLE(2.0, "OccupancyGrid data size mismatch: data=%zu expected=%zu",
                              map_.data.size(), expected);
            have_map_ = false;
            return;
        }

        if (map_.info.resolution <= 0.0)
        {
            ROS_WARN_THROTTLE(2.0, "Invalid map resolution: %.6f", map_.info.resolution);
            have_map_ = false;
            return;
        }

        have_map_ = true;
    }

    void pathCb(const nav_msgs::Path::ConstPtr& msg)
    {
        path_ = *msg;
        have_path_ = !path_.poses.empty();

        if (have_map_ && have_path_)
            evaluateAndPublishScores();
    }

    void healthCb(const std_msgs::Float64::ConstPtr& msg)
    {
        health_ = clamp(msg->data, 0.0, 1.0);
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) const
    {
        if (!have_map_) return false;

        const double origin_x = map_.info.origin.position.x;
        const double origin_y = map_.info.origin.position.y;
        const double res = map_.info.resolution;

        mx = static_cast<int>(std::floor((wx - origin_x) / res));
        my = static_cast<int>(std::floor((wy - origin_y) / res));

        if (mx < 0 || my < 0) return false;
        if (mx >= static_cast<int>(map_.info.width) || my >= static_cast<int>(map_.info.height)) return false;
        return true;
    }

    inline int index(int mx, int my) const
    {
        return my * static_cast<int>(map_.info.width) + mx;
    }

    struct Stats
    {
        double gain_norm = 0.0;   // unknown ratio in neighborhood [0..1]
        double risk_norm = 1.0;   // occupied ratio in neighborhood [0..1]
        double boundary = 0.0;    // simple boundary indicator [0..1]
        bool valid = false;
    };

    Stats computeStatsAt(double wx, double wy) const
    {
        Stats s;

        int cx, cy;
        if (!worldToMap(wx, wy, cx, cy))
        {
            s.valid = false;
            return s;
        }

        const double res = map_.info.resolution;
        const int radius_cells = static_cast<int>(std::ceil(eval_radius_m_ / res));
        const int r2 = radius_cells * radius_cells;

        int unknown_cnt = 0;
        int occupied_cnt = 0;
        int total_cnt = 0;

        // boundary heuristic: count transitions unknown<->known around the center
        // (cheap indicator that you're near frontier-ish area)
        int boundary_hits = 0;
        int boundary_checks = 0;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                if (dx*dx + dy*dy > r2) continue;

                int mx = cx + dx;
                int my = cy + dy;

                if (mx < 0 || my < 0) continue;
                if (mx >= static_cast<int>(map_.info.width) || my >= static_cast<int>(map_.info.height)) continue;

                const int v = map_.data[index(mx, my)];
                total_cnt++;

                if (v == unknown_value_) unknown_cnt++;
                else if (v >= occupied_threshold_) occupied_cnt++;
            }
        }

        if (total_cnt <= 0)
        {
            s.valid = false;
            return s;
        }

        // Frontier-ish boundary heuristic (look at 8-neighborhood around center)
        // If center is known and neighbors are unknown (or vice versa), count as boundary.
        int center_v = map_.data[index(cx, cy)];
        for (int ny = -1; ny <= 1; ++ny)
        {
            for (int nx = -1; nx <= 1; ++nx)
            {
                if (nx == 0 && ny == 0) continue;
                int mx = cx + nx;
                int my = cy + ny;
                if (mx < 0 || my < 0) continue;
                if (mx >= static_cast<int>(map_.info.width) || my >= static_cast<int>(map_.info.height)) continue;

                int nb = map_.data[index(mx, my)];
                boundary_checks++;

                bool c_unknown = (center_v == unknown_value_);
                bool n_unknown = (nb == unknown_value_);

                if (c_unknown != n_unknown) boundary_hits++;
            }
        }

        s.gain_norm = static_cast<double>(unknown_cnt) / static_cast<double>(total_cnt);  // 0..1
        s.risk_norm = static_cast<double>(occupied_cnt) / static_cast<double>(total_cnt); // 0..1
        s.boundary  = (boundary_checks > 0) ? (static_cast<double>(boundary_hits) / boundary_checks) : 0.0;
        s.valid = true;

        return s;
    }

    void evaluateAndPublishScores()
    {
        const auto& poses = path_.poses;
        std_msgs::Float64MultiArray out;
        out.data.resize(poses.size(), -1e9);

        int valid_cnt = 0;

        for (size_t i = 0; i < poses.size(); ++i)
        {
            const double x = poses[i].pose.position.x;
            const double y = poses[i].pose.position.y;

            Stats st = computeStatsAt(x, y);
            if (!st.valid)
            {
                out.data[i] = -1e9;
                continue;
            }

            // Total score
            // - Prefer unknown ratio (gain_norm)
            // - Avoid occupied ratio (risk_norm)
            // - Health adds global stability
            // - Boundary bonus encourages frontier-ish points slightly
            double score =
                (w_gain_   * st.gain_norm) -
                (w_risk_   * st.risk_norm) +
                (w_health_ * health_) +
                (boundary_bonus_ * st.boundary);

            out.data[i] = score;
            valid_cnt++;
        }

        scores_pub_.publish(out);

        ROS_INFO_THROTTLE(1.0, "Published %d/%zu candidate scores (health=%.2f)",
                          valid_cnt, poses.size(), health_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gain_evaluator_node");
    GainEvaluator node;
    ros::spin();
    return 0;
}