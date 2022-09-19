//
// Created by bzdfzfer on 2022/8/26.
//

#ifndef PLANE_PAIRING_H
#define PLANE_PAIRING_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "histogram.h"
#include "fstruct.h"
#include "common.h"

#include <numeric>

class PlanePairing {
public:
    PlanePairing();

    
    void run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& s_norm,  
             const pcl::PointCloud<pcl::PointXYZI>::Ptr& t_norm) {

        normals_s = s_norm;
        normals_t = t_norm;
        matchKdtree();

    }

    void run(std::vector<line>* s_lines_ptr,
             std::vector<line>* t_lines_ptr,
             Histogram* s_hist_ptr,
             Histogram* t_hist_ptr) {

        source_lines = s_lines_ptr;
        target_lines = t_lines_ptr;

        source_hist = s_hist_ptr;
        target_hist = t_hist_ptr;

        matchCoarseToFine();
    }

    std::vector<std::pair<int, int>> getMatchIdxes() { return match_idxes; }
    std::vector<float> getMatchErrors() { return match_errors; }
    void matchKdtree() {}

// ----------------------------------------------------------------------------------
    // COARSE TO FINE MATCHING.
    void matchCoarseToFine();
    double matchHistogramMaxCrossCorrelation(std::vector<std::pair<int,int>>& max_consensus_bin_pairs);
    double computeCrossCorrelation(int di, std::vector<std::pair<int,int>>& bin_pairs, double& psi);
    void votingForConsistentPlanePairs(const double& psi,
        const std::vector<std::vector<int>>& s_plane_sets,
        const std::vector<std::vector<int>>& t_plane_sets);
    std::vector<std::pair<int, int>> votingConsistentDistanceDifference(
        const std::vector<std::pair<int, int>>& potential_pairs);
    
    std::vector<std::pair<int, int>> votingConsistentDistanceDifferenceWithWeights(
        const std::vector<std::pair<int, int>>& potential_pairs,
        std::vector<double>& weights);
    

    // double getPlaneDistanceAt(const std::vector<line>& lines, int line_idx);
    // int getPlaneLeftAt(const std::vector<line>& lines, int line_idx);
    // int getPlaneRightAt(const std::vector<line>& lines, int line_idx);

    bool twoPlaneSimilar(int s_pidx, int t_pidx, double psi, double& over_len);
    std::vector<double> computeDistDiffVecs(const std::vector<std::pair<int,int>>& plane_pairs);
    // std::vector<int> voting1DVec(const std::vector<double>& vec_1d, const double& noise_bound);
    std::vector<int> voting1DVecWithWeights(const std::vector<double>& vec_1d,
        const std::vector<double> &weights, 
        const double& noise_bound);

    int getBinShift() { return delta_bin; }
    std::vector<std::pair<int,int>> 
    enrichMatchedBinPairs(const std::vector<std::pair<int,int>>& bin_pairs, int bin_shift);

    POINT calcFootOfPerpendicular(double la, double lb, double lc, POINT pt);
    bool checkLineOverlap(POINT s_p1, POINT s_p2, POINT ts_p1, POINT ts_p2, double& over_len);



    // setting func.
    void setVotingNoiseBound(double nb) { VOTING_NOISE_BOUND = nb; }
    void setPlaneSimilarRatio(double ratio) { PLANE_SIMILAR_RATIO = ratio; }
    void setHistogramBinNum(int num) { HIST_BIN_NUM = num; }
    void setLaserAngularResolution(double reso) { LASER_ANGULAR_RESOLUTION = reso; }
    void loadParam(double plane_pairing_thresh) { thd = plane_pairing_thresh; }
    void setSearchWindowSize(int size) { SEARCH_WINDOW_SIZE = size; }

    void setMaxDeltaDistance(double dist) { m_max_deltaDist = dist; }

    // input: bin pairs
    // process: 
    //      with initial seed of potential matched region, enlarge matched sets in source and target.
    //
    // output: potential matching plane sets, source and target.

    void absorbNeighborBins(const std::vector<std::pair<int, int>>& bin_pairs,
        std::vector<std::vector<int>>& s_plane_sets, 
        std::vector<std::vector<int>>& t_plane_sets);
    std::vector<int> absorbNeighborBinPlanes(const Histogram& hist, 
        std::vector<bool>& mask,
        int label);

private:
    double thd;

    // input.source_lines
    pcl::PointCloud<pcl::PointXYZI>::Ptr normals_s;
    pcl::PointCloud<pcl::PointXYZI>::Ptr normals_t;

    // output.
    std::vector<std::pair<int, int> > match_idxes;
    std::vector<float> match_errors;

    std::vector<line>* source_lines;
    std::vector<line>* target_lines;
    Histogram* source_hist;
    Histogram* target_hist;

    int maxcc_bin_shift_idx;
    int delta_bin;
    double initial_psi;
    double final_psi;

    double m_max_deltaDist=1.0;

    double LASER_ANGULAR_RESOLUTION;
    int HIST_BIN_NUM;
    int SEARCH_WINDOW_SIZE = 100;   
    double PLANE_SIMILAR_RATIO = 0.5;
    double VOTING_NOISE_BOUND = 0.06;
};

#endif