#include "circle_odometry.h"

CircleOdometry::CircleOdometry() {
    T_sum = Eigen::Matrix3f::Identity();
    s_circle_pts = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    t_circle_pts = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void CircleOdometry::loadParamsFromYAML(const std::string &file_name) {
    std::ifstream fin(file_name);
    if (fin.fail()) {
        cout << ">>>>>  could not open " << file_name.c_str() << endl;
        exit(-1);
    }
    YAML::Node root = YAML::Load(fin);
    fin.close();

    double m_least_thresh;
    double m_min_length;
    double m_predict_distance;
    int m_seed_pts_num;
    int m_min_line_points_num;

    int m_missing_pts_tolerance;
    double m_gap_distance_tolerance;

    double plane_pairing_thresh;

    double m_hist_bin_width, m_hist_bin_max, m_hist_bin_min;
    int hist_search_window_size;
    int hist_absorb_bin_num;
    double plane_similar_ratio;
    double dist_noise_bound;
    float angular_noise_bound;
    bool config_sincos_flag;


    root["m_least_thresh"] >> m_least_thresh;
    root["m_min_length"] >> m_min_length;
    root["m_predict_distance"] >> m_predict_distance;
    root["m_seed_pts_num"] >> m_seed_pts_num;
    root["m_min_line_points_num"] >> m_min_line_points_num;
    root["plane_pairing_thresh"] >> plane_pairing_thresh;

    root["config_sincos_flag"] >> config_sincos_flag;

    root["m_missing_pts_tolerance"] >> m_missing_pts_tolerance;
    root["m_gap_distance_tolerance"] >> m_gap_distance_tolerance;

    cout << "........... Loading params from yaml file: ........." << endl;
    cout << "m_least_thresh: " << m_least_thresh << endl;
    cout << "m_min_length: " << m_min_length << endl;
    cout << "m_predict_distance: " << m_predict_distance << endl;
    cout << "m_seed_pts_num: " << m_seed_pts_num << endl;
    cout << "m_min_line_points_num: " << m_min_line_points_num << endl;
    cout << "plane_pairing_thresh: " << plane_pairing_thresh << endl;
    cout << "config_sincos_flag: " << config_sincos_flag << endl;



    // loading lidar parameters.
    root["m_laser_points_num"] >> m_laser_points_num;
    root["m_laser_angle_min"] >> m_angle_min;
    root["m_laser_angle_max"] >> m_angle_max;
    root["m_laser_angle_incre"] >> m_angle_incre;
    root["m_laser_range_min"] >> m_range_min;
    root["m_laser_range_max"] >> m_range_max;



    cout <<"............ Loading Lidar parameters: ..........." << endl;
    cout <<"m_laser_points_num: " << m_laser_points_num << endl;
    cout <<"m_laser_angle_min: " << m_angle_min << endl;
    cout <<"m_laser_angle_max: " << m_angle_max << endl;
    cout <<"m_laser_angle_incre: " << m_angle_incre << endl;
    cout <<"m_laser_range_min: " << m_range_min << endl;
    cout <<"m_laser_range_max: " << m_range_max << endl;

    cout << endl << "----------------- loading historgram parameters -----------" << endl;
    root["hist_bin_min"] >> m_hist_bin_min;
    root["hist_bin_max"] >> m_hist_bin_max;
    root["hist_bin_width"] >> m_hist_bin_width;
    root["hist_search_window"] >> hist_search_window_size;
    root["hist_absorb_bin_num"] >> hist_absorb_bin_num;
    root["plane_similar_ratio"] >> plane_similar_ratio;
    root["dist_noise_bound"] >> dist_noise_bound;
    root["angular_noise_bound"] >> angular_noise_bound;

    cout << "........... Params loaded ........." << endl;

    pe.loadParams(m_least_thresh, m_min_length, m_predict_distance, 
                  m_range_max, m_seed_pts_num, m_min_line_points_num);
    pe.setLaserPointsNum(m_laser_points_num);
    pe.setLaserAngleStart(m_angle_min);
    pe.setLaserAngleStep(m_angle_incre);
    if(config_sincos_flag) 
        pe.computeBearingSinCosFromParam();

    pe.setLineFeatureMaxPtsGap(m_gap_distance_tolerance);
    pe.setLineFeaturePtsMissingTolerance(m_missing_pts_tolerance);

    pp.loadParam(plane_pairing_thresh);



    pe.setHistogramBinMinMax(m_hist_bin_min, m_hist_bin_max, m_hist_bin_width);
    pe.setHistAbosrbBinNum(hist_absorb_bin_num);
    
    pp.setHistogramBinNum(pe.getHistogram().getBinNum());
    pp.setLaserAngularResolution(m_angle_incre*180.0/M_PI);
    pp.setSearchWindowSize(hist_search_window_size);
    pp.setPlaneSimilarRatio(plane_similar_ratio);
    pp.setVotingNoiseBound(dist_noise_bound);

    double max_delta_dist;
    if(root["max_delta_dist"]) {
        root["max_delta_dist"] >> max_delta_dist;
        pp.setMaxDeltaDistance(max_delta_dist);
        cout << "max_delta_dist set to: " << max_delta_dist << endl;
    }


    s_polar.setAngularNoiseBound(angular_noise_bound*M_PI/180.0);

    float eigen_threshold1, eigen_threshold2;
    if(root["eigen_threshold1"]) {
        root["eigen_threshold1"] >> eigen_threshold1;
        root["eigen_threshold2"] >> eigen_threshold2;

        s_polar.setDegenerateEigenThreshold(eigen_threshold1, eigen_threshold2);

        cout << "Degenerate: eigen_threshold1 set to: " << eigen_threshold1 << endl;
        cout << "Degenerate: eigen_threshold2 set to: " << eigen_threshold2 << endl;
    }

}

void CircleOdometry::run(const pcl::PointCloud<pcl::PointXY>& cloudIn) {
    tCloudPtr = cloudIn.makeShared();

    // pe.handleScan(*tCloudPtr);
    // // pe.handleScan(scan_msg);

    // auto pl_norms = pe.getPlanesNormAndDists();
    // tNormalPtr = pl_norms.makeShared();

    process();
}

void CircleOdometry::run(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    pcl::PointCloud<pcl::PointXY> laserCloudIn;
    sensor_msgs::PointCloud2 cloud2msg;
    fromLaserScanToCloud2Msg(scan_msg, cloud2msg);
    pcl::fromROSMsg(cloud2msg, laserCloudIn);
    tCloudPtr = laserCloudIn.makeShared();

    process();
}

void CircleOdometry::align(const pcl::PointCloud<pcl::PointXY>& source_scan, 
                    const pcl::PointCloud<pcl::PointXY>& target_scan) {

    cout << "===============================================" << endl;
    frame_cnt = 0;
    T_sum = Eigen::Matrix3f::Identity();
    
    tCloudPtr = source_scan.makeShared();
    
    // process source scan.
    process();

    m_source_dist_hist.clear();
    m_source_theta_hist.clear();

    if(tNormalPtr->size() != t_pidxs.size()) {
        cout << "[CO-256] the size of NormalDists: " << tNormalPtr->size() << endl;
        cout << "[CO-256] the size of plane indexes:" << t_pidxs.size() << endl;
    }

    for(int i=0; i < tNormalPtr->size(); i++) {
        float dis = tNormalPtr->points[i].intensity;
        float theta = std::atan2(tNormalPtr->points[i].y, tNormalPtr->points[i].x);
        for(int j= t_pidxs[i].first; j <= t_pidxs[i].second; j++) {
            pcl::PointXYZ pt;
            pt.x = (float)j/100.0;
            pt.y = dis -15;
            pt.z = 0.0;
            m_source_dist_hist.push_back(pt);

            pt.y = theta-20;
            m_source_theta_hist.push_back(pt);
        }
    }

    tCloudPtr = target_scan.makeShared();

    // process target scan and align.
    process();

    m_target_dist_hist.clear();
    m_target_theta_hist.clear();

    if(tNormalPtr->size() != t_pidxs.size()) {
        cout << "[CO-256] the size of NormalDists: " << tNormalPtr->size() << endl;
        cout << "[CO-256] the size of plane indexes:" << t_pidxs.size() << endl;
    }

    for(int i=0; i < tNormalPtr->size(); i++) {
        float dis = tNormalPtr->points[i].intensity;
        float theta = std::atan2(tNormalPtr->points[i].y, tNormalPtr->points[i].x);
        for(int j= t_pidxs[i].first; j <= t_pidxs[i].second; j++) {
            pcl::PointXYZ pt;
            pt.x = (float)j/100.0;
            pt.y = dis-15;
            pt.z = 0.0;
            m_target_dist_hist.push_back(pt);

            pt.y = theta-20;
            m_target_theta_hist.push_back(pt);
        }
    }

    return;
}


void CircleOdometry::process() {
    frame_cnt ++;       
    
    
    TicToc timer;
    timer.Tic();
    // 1. plane extraction.
    pe.handleScan(*tCloudPtr);
    // pe.handleScan(scan_msg);
    double pe_time = timer.Toc();
    // cout << "plane extraction runtime: " << timer.Toc()  << "ms" << endl;

    auto pl_norms = pe.getPlanesNormAndDists();
    tNormalPtr = pl_norms.makeShared();


    t_lines = pe.getLines();
    m_g_lines = pe.getGlines();

    t_pidxs = pe.getPlaneIdxs();

    target_hist = pe.getHistogram();

    t_g_lines = m_g_lines;


    if(frame_cnt == 1) {
        // cout << "first frame set" << endl;
        sCloudPtr = tCloudPtr;
        sNormalPtr = tNormalPtr;
        s_circle_pts = t_circle_pts;
        s_lines = t_lines;
        s_g_lines = t_g_lines;
        source_hist = target_hist;
        s_pidxs = t_pidxs;

        return;
    }


    timer.Tic();

    // 2. plane pairing.
    pp.run(&s_lines, &t_lines, &source_hist, &target_hist);
    double pm_time = timer.Toc();

    std::vector<std::pair<int, int>> matches = pp.getMatchIdxes();


    m_matches = matches;
    vis_s_g_lines = s_g_lines;
    vis_t_g_lines = t_g_lines;

    vis_s_hist = source_hist;
    vis_t_hist = target_hist;


    if(matches.size() < 1)
    {
        cout << "bad matching .... using last T as current transform" << endl;
        sCloudPtr = tCloudPtr;
        sNormalPtr = tNormalPtr;
        s_lines = t_lines;
        s_g_lines = t_g_lines;
        source_hist = target_hist;
        s_pidxs = t_pidxs;

        T_sum = T_sum*T_s_t;
        return;
    }

    s_circle_pts->clear();
    t_circle_pts->clear();
    for(int i=0; i<sNormalPtr->size(); i++) {
        pcl::PointXYZ pt;
        pt.z = 0;
        pt.x = sNormalPtr->points[i].intensity * sNormalPtr->points[i].x;
        pt.y = sNormalPtr->points[i].intensity * sNormalPtr->points[i].y;

        s_circle_pts->points.push_back(pt);
    }
    for(int i=0; i<tNormalPtr->size(); i++) {
        pcl::PointXYZ pt;
        pt.z = 0;
        pt.x = tNormalPtr->points[i].intensity * tNormalPtr->points[i].x;
        pt.y = tNormalPtr->points[i].intensity * tNormalPtr->points[i].y;
      
        t_circle_pts->points.push_back(pt);
    }


    // 4. motion estimation.
    // set weights first.
    // s_polar.setWeights(pp.getMatchErrors(), matches, s_circle_pts, t_circle_pts);
    s_polar.setWeights(pp.getMatchErrors() );
    timer.Tic();
    s_polar.run(sNormalPtr, tNormalPtr, matches);
    // cout << "motion estimation runtime: " << timer.Toc() << "ms" << endl;
    double solver_time = timer.Toc();

    // cout << "[CL-104] motion estimated ...." << endl;

    pe_times.push_back(pe_time);
    pm_times.push_back(pm_time);
    solver_times.push_back(solver_time);


    headingAngle = s_polar.getHeadingAngle();
    t_s_t = s_polar.getTranslation();

    T_s_t = s_polar.getTransform();

    T_sum = T_sum*T_s_t;



    // assign last frame
    sCloudPtr = tCloudPtr;
    sNormalPtr = tNormalPtr;
    s_lines = t_lines;
    s_g_lines = t_g_lines;
    source_hist = target_hist;

    s_pidxs = t_pidxs;

}


inline void saveTimesToFile(std::string out_file, std::vector<double> times_vec) {
    std::cout << "saving timings to file : " << out_file  << " ...." << std::endl;
    FILE* fptw = NULL;
    fptw = fopen(out_file.c_str(), "w");
    if(fptw == NULL) {
        std::cout << "cannot open " << out_file << std::endl;
        return;
    }

    for(int i=0; i < times_vec.size(); i++) {
        fprintf(fptw, "%lf\n", times_vec[i]);
    }
    fclose(fptw);
    std::cout << "timing file saved ..." << std::endl;    
}

void CircleOdometry::saveTimingFile(std::string file_path, std::string file_name) {
    std::string pe_file = file_path + file_name + "_pe.txt";
    saveTimesToFile(pe_file, pe_times);
    std::string pm_file = file_path + file_name + "_pm.txt";
    saveTimesToFile(pm_file, pm_times);
    std::string solver_file = file_path + file_name + "_solver.txt";
    saveTimesToFile(solver_file, solver_times);
}