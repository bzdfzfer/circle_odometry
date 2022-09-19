//
// Created by bzdfzfer on 2022/8/26.
//


#ifndef CIRCILE_ODOMETRY_H
#define CIRCILE_ODOMETRY_H

#include <stdio.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include "plane_pairing.h"
#include "solver.h"
// #include "Visualizer.h"
#include "histogram.h"
#include "plane_extraction.h"
#include "common.h"

#include "TicToc.h"


using std::cout;
using std::endl;
using std::cos;
using std::sin;

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

class CircleOdometry {
public:
    CircleOdometry();

    void loadParamsFromYAML(const std::string& file_name);
    
    void run(const pcl::PointCloud<pcl::PointXY>& cloudIn);

    void run(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void align(const pcl::PointCloud<pcl::PointXY>& source_scan, 
                const pcl::PointCloud<pcl::PointXY>& target_scan);

    void process();

    Eigen::Matrix3f getTransform() { return T_s_t; }
    Eigen::Matrix3f getTransformSum() { return T_sum; }

    std::vector<gline> getSourceGlinesVis() { return vis_s_g_lines; }
    std::vector<gline> getTargetGlinesVis() { return vis_t_g_lines; }

    std::vector<std::pair<int, int>> getMatches() { return m_matches; }

    double getLidarAngleMin() { return m_angle_min; }
    double getLidarAngleMax() { return m_angle_max; }
    double getAngleIncrement() { return m_angle_incre; }
    double getLidarRangeMin() { return m_range_min; }
    double getLidarRangeMax() { return m_range_max; }

    pcl::PointCloud<pcl::PointXYZ> getSourceDistHist() { return m_source_dist_hist; }
    pcl::PointCloud<pcl::PointXYZ> getSourceThetaHist() { return m_source_theta_hist; }
    pcl::PointCloud<pcl::PointXYZ> getTargetDistHist() { return m_target_dist_hist; }
    pcl::PointCloud<pcl::PointXYZ> getTargetThetaHist() { return m_target_theta_hist; }


    Histogram getSourceHistogram() { return vis_s_hist; }
    Histogram getTargetHistogram() { return vis_t_hist; }

    Histogram getTargetShiftedHistogram() {
        Histogram* shifted_hist_ptr;
        shifted_hist_ptr = Histogram::createShiftedHistogram(vis_t_hist, pp.getBinShift());
        return *shifted_hist_ptr;
    }

    void saveTimingFile(std::string file_path, std::string file_name);

    PlanePairing pp;

    PlaneExtraction pe;

    Solver2d_polar s_polar;

    // PlaneNormalVisualizer normal_vis;


private:
    // input.
    pcl::PointCloud<pcl::PointXY>::Ptr tCloudPtr;
    pcl::PointCloud<pcl::PointXY>::Ptr sCloudPtr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sCloudNormalPtr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tCloudNormalPtr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr sNormalPtr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tNormalPtr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr s_circle_pts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr t_circle_pts;

    std::vector<std::pair<int, int>> s_pidxs;
    std::vector<std::pair<int, int>> t_pidxs;



    // middle variables.
    std::vector<line> s_lines;
    std::vector<line> t_lines;
    std::vector<gline> m_g_lines;
    std::vector<gline> s_g_lines;
    std::vector<gline> t_g_lines;
    std::vector<gline> vis_s_g_lines;
    std::vector<gline> vis_t_g_lines;

    std::vector<std::pair<int, int>> m_matches;


    int last_pair_num = 0;


    pcl::PointCloud<pcl::PointXYZ> m_source_dist_hist;
    pcl::PointCloud<pcl::PointXYZ> m_source_theta_hist;
    pcl::PointCloud<pcl::PointXYZ> m_target_dist_hist;
    pcl::PointCloud<pcl::PointXYZ> m_target_theta_hist;

    //output.
    float headingAngle;
    Eigen::Matrix2f R_s_t;
    Eigen::Vector2f t_s_t;
    Eigen::Matrix3f T_s_t;

    Eigen::Matrix3f T_sum;

    int frame_cnt = 0;

    Histogram source_hist;
    Histogram target_hist;

    Histogram vis_s_hist;
    Histogram vis_t_hist;

    // Lidar parameters.
    double m_laser_points_num;
    double m_angle_min;
    double m_angle_max;
    double m_angle_incre;
    double m_range_min;
    double m_range_max;

    std::vector<double> pe_times;
    std::vector<double> pm_times;
    std::vector<double> solver_times;
};




#endif