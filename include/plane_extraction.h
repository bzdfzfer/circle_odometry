//
// Created by bzdfzfer on 2022/8/26.
//

#ifndef PLANE_EXTRACTION_H
#define PLANE_EXTRACTION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Visualizer.h"

#include <sensor_msgs/LaserScan.h>

#include "line_feature.h"
#include "histogram.h"
#include "common.h"

#include <ros/ros.h>

#include <iostream>

using std::cout;
using std::endl;

using line_feature::LineFeature;

class PlaneExtraction {
public:
    PlaneExtraction();

    void process();

    void handleScan(const sensor_msgs::LaserScan::ConstPtr& );
    void handleScan(const pcl::PointCloud<pcl::PointXY> & cloud_in);

    void loadParams(double least_thd, 
                    double min_length, 
                    double pred_dist, 
                    double max_range, 
                    int seed_num, 
                    int min_line_pts_num);

    void setLaserPointsNum(int num) {
        lf.set_laser_points_num(num);
    }

    void setLaserAngleStart(double ang) {
        lf.set_angle_start(ang);
    }

    void setLaserAngleStep(double ang) {
        lf.set_angle_increment(ang);
    }

    void setLineFeatureMaxPtsGap(double gap_dist) {
        lf.set_max_pts_gap(gap_dist);
    }

    void setLineFeaturePtsMissingTolerance(int missing_num) {
        lf.set_pts_missing_tolerance(missing_num);
    }
    

    int getLaserPointsNum() {
        return lf.get_laser_points_num();
    }

    double getLaserAngleStart() {
        return lf.get_angle_start();
    }

    double getLaserAngleStep() {
        return lf.get_angle_increment();
    }

    void setHistAbosrbBinNum(int num) {
        plane_angle_hist.setAbsorbBinNum(num);
    }

    pcl::PointCloud<pcl::PointXYZI> getPlanesNormAndDists() { return *normalOut; }
    std::vector<double> getWeights() { return weights; }
    std::vector<gline> getGlines() { return m_gline; }
    std::vector<line> getLines() { return m_line; }
    
    std::vector<std::pair<int, int>> getPlaneIdxs() { return m_line_idxs; }
    // std::vector<std::pair<float, float>> getPlaneMidCosSins() { return m_line_mid_cossins; }
    // std::vector<float> getPlaneMidDists() { return m_line_mid_dists; }
    // std::vector<std::pair<float, float>> getPlaneMidPoints() {
    //     return m_line_mid_points;
    // }

    void setHistogramBinMinMax(double min, double max, double width) {
        plane_angle_hist.setBinMaxMinWidth(max, min, width);
    }
    Histogram getHistogram() {
        return plane_angle_hist;
    }
    bool isDegenerated() { return m_degenerate_flag; }
    void setDegenerateAngleThreshold(double thres) { m_degenerate_threshold = thres; }
    void analyseHistogram();

    PlaneNormalVisualizer normal_vis;

    void computeBearingSinCosFromParam();

private:
    void computeBearingSinCos(const sensor_msgs::LaserScan::ConstPtr&);

    // get vector of plane start and stop indexes.
    std::vector<std::pair<int, int>> computePlaneIdxs();

    // std::vector<std::pair<float, float>> computePlaneMidCosSins();     
    
    // std::vector<std::pair<float, float>> computePlaneMidPoints();

    // std::vector<float> computePlaneMidDists();    

    void histogramCounting();    

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr normalOut;
    std::vector<double> weights;

    LineFeature lf;
    Histogram plane_angle_hist;

    std::vector<gline> m_gline;
    std::vector<line> m_line;

    std::vector<std::pair<int, int>> m_line_idxs;
    // std::vector<std::pair<float, float>> m_line_mid_cossins;
    // std::vector<float> m_line_mid_dists;
    // std::vector<std::pair<float, float>> m_line_mid_points;
    
    std::vector<std::pair<float, float>> m_line_dist_thetas;

    bool bearing_set_flag = false;
    double m_angleStart;
    double m_angleStep;

    // parameters settings for LineFeature.
    double m_least_thresh; //default  = 0.04
    double m_min_length; //default = 0.3
    double m_predict_distance; //default = 0.1
    double m_max_range;
    int m_seed_pts_num; //default = 8
    int m_min_line_points_num; //default = 40, 1 deg 5 pts for pepperlfuch.


    // check degenerated scenes by analysing histogram.
    bool m_degenerate_flag;
    double m_degenerate_threshold = 15.0;
};    

#endif