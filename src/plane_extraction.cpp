//
// Created by bzdfzfer on 2022/8/26.
//

#include "plane_extraction.h"

PlaneExtraction::PlaneExtraction() {

    normalOut = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>() );
}

void PlaneExtraction::loadParams(double least_thd, double min_length, double pred_dist,
                                 double max_range,
                                 int seed_num, int min_line_pts_num) {
    m_least_thresh = least_thd;
    m_min_length = min_length;
    m_predict_distance = pred_dist;
    m_max_range = max_range;
    m_seed_pts_num = seed_num;
    m_min_line_points_num = min_line_pts_num;

    lf.set_least_threshold(m_least_thresh);
    lf.set_min_line_length(m_min_length);
    lf.set_predict_distance(m_predict_distance);
    lf.set_seed_line_points(m_seed_pts_num);
    lf.set_min_line_points(m_min_line_points_num);
    lf.set_max_range(m_max_range);
}

void PlaneExtraction::handleScan(const sensor_msgs::LaserScan::ConstPtr & scan_msg) {
    if(!bearing_set_flag) {
        computeBearingSinCos(scan_msg);
        bearing_set_flag = true;
        cout << " bearing angles sin cos set !!!///" << endl;
        cout << " ----bearing angle size: " << lf.getBearingAngleSize() << endl;
        cout << "line_feature parameters: " << endl;
        cout << "----angle_increment: " <<  lf.get_angle_increment() << endl;
        cout << "----angle_start: " << lf.get_angle_start() << endl;
        cout << "----least_thresh: " << lf.get_least_thresh() << endl;
        cout << "----line_length: " << lf.get_line_length() << endl;
        cout << "----predict_distance: " << lf.get_predict_distance() << endl;
        cout << "----min_line_points: " << lf.get_min_line_points() << endl;
        cout << "----seed_line_points: " << lf.get_seed_line_points() << endl;       

    }
    std::vector<double> scan_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
    // set input data.
    lf.setRangeData(scan_ranges);

//  cout << "range data size in PlaneExtraction: " << lf.getRangeDataSize() << endl;

    process();
}

void PlaneExtraction::handleScan(const pcl::PointCloud<pcl::PointXY> & cloud_in) {

    // initialize CSdata in linefeature.
    if(!bearing_set_flag) {
        bearing_set_flag = true;


        m_angleStart = -3.14159;
        double angle_stop = - m_angleStart;
        m_angleStep = (angle_stop - m_angleStart)/(cloud_in.size()-1);
        cout << "angle start" << m_angleStart << ", angle stop: " << angle_stop << ", step: " << m_angleStep << endl;

        // for pepperl laser scanner.
        m_angleStep = -3.14159274101;
        angle_stop = - m_angleStart;
        m_angleStep = 0.00349065847695;
        // cout << "angle start" << m_angleStart << ", angle stop: " << angle_stop << ", step: " << m_angleStep << endl;

        lf.set_angle_start(m_angleStart);
        lf.set_angle_increment(m_angleStep);

        std::vector<double> bearings, cos_bearings, sin_bearings;
        std::vector<unsigned int> indexes;
        double b = m_angleStart;
        for(int i=0; i < cloud_in.size(); i++,  b += m_angleStep) {
            bearings.push_back(b);
            cos_bearings.push_back(cos(b));
            sin_bearings.push_back(sin(b));
            indexes.push_back(i++);
        }
        for(int i=1; i < cloud_in.size(); i++, b + m_angleStep) {
            bearings.push_back(b);
            cos_bearings.push_back(cos(b));
            sin_bearings.push_back(sin(b));
            indexes.push_back(i++);            
        }
        lf.setCosSinData(bearings, cos_bearings, sin_bearings, indexes);
    }

    // set input data.
    std::vector<double> xs, ys;
    for(int i=0; i < cloud_in.size(); i++) {
        xs.push_back(cloud_in[i].x);
        ys.push_back(cloud_in[i].y);
    }
    lf.setRangeData(xs, ys);

    // cout << "range data size in PlaneExtraction: " << lf.getRangeDataSize() << endl;

    process();
}


// input: m_line, output: plane angle histogram.
void PlaneExtraction::histogramCounting() {
    std::vector<double> ang_vec;
    for(int i=0; i < m_line.size(); i++) {
        double theta = m_line[i].theta;

        if(theta < -M_PI/2)
            theta += M_PI;
        else if(theta >= M_PI/2)
            theta -= M_PI;
        
        theta *= RAD2DEG_RATIO;
        ang_vec.push_back(theta);
    }
    plane_angle_hist.process(ang_vec);
}

std::vector<std::pair<int, int>> PlaneExtraction::computePlaneIdxs() {
    std::vector<std::pair<int, int>> start_stops;
    for(int i=0; i < m_line.size(); i++) {
        std::pair<int, int> start_stop;
        start_stop.first = m_line[i].left;
        start_stop.second = m_line[i].right;
        start_stops.push_back(start_stop);
        // cout << "line id: [" << i << "]: " << start_stop.first << ", " << start_stop.second << endl;
    }
    return start_stops;
}

void PlaneExtraction::process() {

    lf.extractLines(m_line, m_gline);

    // histogram process.
    histogramCounting();

    analyseHistogram();

    // compute start and stop idxs of all planes
    m_line_idxs = computePlaneIdxs();
    // m_line_mid_cossins = computePlaneMidCosSins();
    // m_line_mid_dists = computePlaneMidDists();
    // m_line_mid_points = computePlaneMidPoints();

    // todo: done.
    // convert to pcl::PointCloud<pcl::PointXYZI>
    normalOut->clear();
    weights.clear();
    for(int i=0; i < m_line.size(); i ++) {

        pcl::PointXYZI pt;
        pt.x = m_line[i].nvp.x;
        pt.y = m_line[i].nvp.y;
        pt.z = 0;
        pt.intensity = m_line[i].vdist;
        normalOut->push_back(pt);
        weights.push_back(1/m_line[i].var);
    }

    // visualization
    if(normal_vis.init) {
        // show point clouds.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<double> xs, ys;
        lf.getXYData(xs, ys);
        for(int i=0; i < xs.size(); i++) {
            pcl::PointXYZ pt;
            if(std::isnan(xs[i]) || std::isnan(ys[i]))
                continue;

            pt.x = xs[i]; pt.y = ys[i]; pt.z = 0;
            cloud_ptr->points.push_back(pt);
        }
        normal_vis.UpdateCloud(cloud_ptr);

        // show lines.
        pcl::PointCloud<pcl::PointXYZ>::Ptr line_end_pts1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr line_end_pts2(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0; i < m_gline.size(); i++) {
            double mag = sqrt(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            if(std::isnan(m_line[i].c) || fabs(m_line[i].c/mag) < 0.5 )
                continue;

            if(m_line[i].var > 0.02)
                continue;

            pcl::PointXYZ pt1, pt2;
            pt1.z = pt2.z = 0;
            pt1.x = m_gline[i].x1;
            pt1.y = m_gline[i].y1;
            line_end_pts1->points.push_back(pt1);
            pt2.x = m_gline[i].x2;
            pt2.y = m_gline[i].y2;
            line_end_pts2->points.push_back(pt2);
        }

        for(int i=0; i < normalOut->points.size(); i++) {
            pcl::PointXYZ pt1, pt2;
            pt1.x = pt1.y = pt1.z = pt2.z = 0;   
            line_end_pts1->points.push_back(pt1);
            pt2.x = normalOut->points[i].intensity * normalOut->points[i].x;
            pt2.y = normalOut->points[i].intensity * normalOut->points[i].y;
            line_end_pts2->points.push_back(pt2);            
        }

        normal_vis.UpdateLines(line_end_pts1, line_end_pts2);
    }


}

// input: plane angle histogram: 
//           here we just need two identify the angular relationship between valid bins.
// output: degenerate_flag.
void PlaneExtraction::analyseHistogram() {
    m_degenerate_flag = false;

    int VBN = plane_angle_hist.getValidBinNum();
    int BCN = plane_angle_hist.getBinClusterNum();
    // only one single bin, one plane, this is degenerated case.
    if(VBN <= 1 || BCN <= 1) {
        m_degenerate_flag = true;
        return;
    }

    // larger than (or equal to) 2 bin clusters.
    std::vector<int> valid_bins = plane_angle_hist.getValidBins();
    std::vector<int> valid_bin_labels = plane_angle_hist.getValidBinLabels();

    // compute center of bin sets with same label.
    std::vector<double> cluster_centers(BCN);

    for(int i=0; i < BCN; i++) {
        double ci = cluster_centers[i];
        for(int j=i+1; j < BCN; j++) {
            double cj = cluster_centers[j];
            double dcij = cj - ci;
            // two bin sets offer good orthognal constraint, this is not degenerated case.
            if(dcij > m_degenerate_threshold && dcij < (180.0-m_degenerate_threshold)) { 
                return;
            }
        }
    }

    // above cases are not guaranteed.
    m_degenerate_flag = true;

    return;
}

void PlaneExtraction::computeBearingSinCos(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    m_angleStart = scan_msg->angle_min;
    m_angleStep = scan_msg->angle_increment;

    cout << "scan angle my_min: " << m_angleStart << endl;
    cout << "scan angle step: " << m_angleStep << endl;
    cout << "scan angle my_max: " << scan_msg->angle_max << endl;
    // cout << "scan points num: " << scan_msg->ranges.size() << endl;

    lf.set_angle_start(m_angleStart);
    lf.set_angle_increment(m_angleStep);

    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indexes;
    unsigned  int i=0;
    for(double b = m_angleStart; b <= scan_msg->angle_max; b += m_angleStep) {
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indexes.push_back(i++);
    }

    lf.setCosSinData(bearings, cos_bearings, sin_bearings, indexes);
}

void PlaneExtraction::computeBearingSinCosFromParam() {
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indexes;

    int laser_pts_num = getLaserPointsNum();
    m_angleStart = getLaserAngleStart();
    m_angleStep = getLaserAngleStep(); //lf.get_angle_increment();

    unsigned  int i=0;
    for(double b = m_angleStart; i < laser_pts_num; b += m_angleStep) {
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indexes.push_back(i++);
    }



    lf.setCosSinData(bearings, cos_bearings, sin_bearings, indexes);
    bearing_set_flag = true;

    cout << " bearing angles sin cos set manully !!!///" << endl;
    cout << " ----bearing angle size: " << lf.getBearingAngleSize() << endl;
    cout << "scan angle my_min: " << m_angleStart << endl;
    cout << "scan angle step: " << m_angleStep << endl;    
}