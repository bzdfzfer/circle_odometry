//
// Created by bzdfzfer on 2022/8/26.
//

#include "plane_pairing.h"

PlanePairing::PlanePairing() {
    // set threshold
//  thd = 0.05;
    LASER_ANGULAR_RESOLUTION = 360.0/1800; // 0.2 deg. 
}


void PlanePairing::matchCoarseToFine() {
    std::vector<std::pair<int,int>> bin_pairs;
    initial_psi = matchHistogramMaxCrossCorrelation(bin_pairs);

    delta_bin = maxcc_bin_shift_idx - SEARCH_WINDOW_SIZE/2;

    // input: matched bin pairs, dleta bin shift. 
    // output: more matched bin pairs.
    bin_pairs = enrichMatchedBinPairs(bin_pairs, delta_bin);

    std::vector<std::vector<int>> s_plane_sets, t_plane_sets;
    absorbNeighborBins(bin_pairs, s_plane_sets, t_plane_sets);

    votingForConsistentPlanePairs(initial_psi, s_plane_sets, t_plane_sets); 
}


std::vector<std::pair<int,int>> 
 PlanePairing::enrichMatchedBinPairs(const std::vector<std::pair<int,int>>& bin_pairs, int bin_shift)
{
    std::vector<std::pair<int,int>> enriched_pairs;

    // 1. get unmatched source target bins.
    enriched_pairs = bin_pairs;
    std::vector<int> unmatched_s_bin_idxs;
    unmatched_s_bin_idxs = source_hist->getValidBins();
    for(int i=0; i < unmatched_s_bin_idxs.size(); i++) {
        int unmatched_s_bin_idx = unmatched_s_bin_idxs[i];
        for(int j=0; j < enriched_pairs.size();j++) {
            int matched_s_bin_idx = enriched_pairs[j].second;
            if(unmatched_s_bin_idx == matched_s_bin_idx) {
                unmatched_s_bin_idxs.erase(unmatched_s_bin_idxs.begin() + i);
                enriched_pairs.erase(enriched_pairs.begin() + j);
                break;
            }
        }
    }

    enriched_pairs = bin_pairs;
    std::vector<int> unmatched_t_bin_idxs;
    unmatched_t_bin_idxs = target_hist->getValidBins();
    for(int i=0; i < unmatched_t_bin_idxs.size(); i++) {
        int unmatched_t_bin_idx = unmatched_t_bin_idxs[i];
        for(int j=0; j < enriched_pairs.size();j++) {
            int matched_t_bin_idx = enriched_pairs[j].second;
            if(unmatched_t_bin_idx == matched_t_bin_idx) {
                unmatched_t_bin_idxs.erase(unmatched_t_bin_idxs.begin() + i);
                enriched_pairs.erase(enriched_pairs.begin() + j);
                break;
            }
        }
    }

    // 2.  obtain enriched bin pairs.
    enriched_pairs = bin_pairs;
    for(int i=0; i < unmatched_s_bin_idxs.size(); i++) {
        int s_bin_idx = unmatched_s_bin_idxs[i];
        for(int j=0; j < unmatched_t_bin_idxs.size(); j++) {
            int t_bin_idx = unmatched_t_bin_idxs[j];
            int shifted_nt_bin_idx = wrap_idx(t_bin_idx+bin_shift+1, HIST_BIN_NUM);
            int shifted_pt_bin_idx = wrap_idx(t_bin_idx+bin_shift-1, HIST_BIN_NUM);
            if(s_bin_idx == shifted_pt_bin_idx || s_bin_idx == shifted_nt_bin_idx) {
                enriched_pairs.push_back({s_bin_idx, t_bin_idx});
                break;
            }
        }
    }
    return enriched_pairs;
}

// input: matched bin pairs, 
// output: merged and enlarged matching pairs.
std::vector<int> PlanePairing::absorbNeighborBinPlanes(const Histogram& hist,
                        std::vector<bool>& mask,
                        int label) {
    std::vector<int> plane_idxs;
    for(int i=0; i < hist.getValidBinNum(); i++) {
        if(mask[i])
            continue;
        int bin_idx = hist.getValidBinIdxAt(i);
        int bin_label = hist.getValidBinLabelAt(i);
        if(bin_label == label) {
            mask[i] = true;

            std::vector<int> bin_pidxs = hist.getBinIndexesAt(bin_idx);
            plane_idxs.insert(plane_idxs.end(), bin_pidxs.begin(), bin_pidxs.end());
        }
    }
    return plane_idxs;  
}

void PlanePairing::absorbNeighborBins(const std::vector<std::pair<int, int>>& bin_pairs,
        std::vector<std::vector<int>>& s_plane_sets, 
        std::vector<std::vector<int>>& t_plane_sets)
{

    int s_valid_bin_num, t_valid_bin_num;
    s_valid_bin_num = source_hist->getValidBinNum();
    t_valid_bin_num = target_hist->getValidBinNum();

    std::vector<bool> s_mask(s_valid_bin_num, false);
    std::vector<bool> t_mask(t_valid_bin_num, false);



    for(int i=0; i < bin_pairs.size(); i++) {
        int s_bin_idx = bin_pairs[i].first;
        int t_bin_idx = bin_pairs[i].second;

        int s_bin_label = source_hist->getBinLabelAt(s_bin_idx);
        int t_bin_label = target_hist->getBinLabelAt(t_bin_idx);

        std::vector<int> s_plane_idxs, t_plane_idxs;
        s_plane_idxs = absorbNeighborBinPlanes(*source_hist, s_mask, s_bin_label);
        t_plane_idxs = absorbNeighborBinPlanes(*target_hist, t_mask, t_bin_label);
        if(s_plane_idxs.size()>0 && t_plane_idxs.size()>0 ) {
            s_plane_sets.push_back(s_plane_idxs);
            t_plane_sets.push_back(t_plane_idxs);           
        }
    }

}

// Using a coarse bin width (such as 1 deg) to cluster parallel lines.
// Then, match against two histograms to align bins, this is done by maximizing cross-correlations.
// input: source histogram and target histogram.
// output: matched bin pairs and rotation angle psi.
double PlanePairing::matchHistogramMaxCrossCorrelation(
    std::vector<std::pair<int,int>>& max_consensus_bin_pairs) {

    max_consensus_bin_pairs.clear();
    
    double max_cc = 0;
    
    maxcc_bin_shift_idx = -1;

    double max_psi = 0.0;

    std::vector<double> valid_ccs;
    std::vector<double> valid_psis;
    std::vector<std::vector<std::pair<int, int>>> valid_pairs_sets;
    for(int ci=0; ci <= SEARCH_WINDOW_SIZE; ci++) {
        int di = ci - SEARCH_WINDOW_SIZE/2;
        std::vector<std::pair<int,int>> bin_pairs;
        double psi;
        double cc = computeCrossCorrelation(di, bin_pairs, psi);
        if( cc > 0.1) {
            valid_ccs.push_back(cc);
            valid_psis.push_back(psi);
            valid_pairs_sets.push_back(bin_pairs);
        }
        if( cc > max_cc) {
            max_cc = cc;
            maxcc_bin_shift_idx = ci;
            max_consensus_bin_pairs = bin_pairs;
            max_psi = psi;
        }
    }

    // check the best match cases.
    int max_cc_num = 0;
    double min_psi_shift = std::fabs(max_psi);
    int min_shift_idx = -1;
    for(int i=0; i < valid_ccs.size(); i++) {
        if(valid_ccs[i] != max_cc)
            continue;
        max_cc_num ++;
        if(std::fabs(valid_psis[i]) < min_psi_shift) {
            min_psi_shift = std::fabs(valid_psis[i]);
            min_shift_idx = i;
        }
    }

    if(min_shift_idx > 0 ) {
        max_consensus_bin_pairs = valid_pairs_sets[min_shift_idx];
        max_psi = valid_psis[min_shift_idx];
    }

    // return this value 
    return max_psi;
}

// input source histogram, target histogram.
// output: cross correlation score, matched pairs, rotation angle.
double PlanePairing::computeCrossCorrelation(int di, 
    std::vector<std::pair<int,int>>& bin_pairs, 
    double& psi) {

    double cross_correlation_score = 0;
    bin_pairs.clear();
    psi = 0;
    double match_cnt = 0;
    std::vector<int> target_valid_bins = target_hist->getValidBins();

    for(int i=0; i < target_valid_bins.size(); i++) {

        int target_bin_idx = target_valid_bins[i];
        int source_bin_match_idx = target_bin_idx + di;

        // this is for 360 deg scan.
        // TODO: handle scan with 180 deg, or 270 deg.
        // if(source_bin_match_idx < 0)
        //  source_bin_match_idx += HIST_BIN_NUM;
        // else if(source_bin_match_idx >= HIST_BIN_NUM)
        //  source_bin_match_idx -= HIST_BIN_NUM;
        source_bin_match_idx = wrap_idx(source_bin_match_idx, HIST_BIN_NUM);

        int source_nbin_match_idx = wrap_idx(target_bin_idx + di + 1, HIST_BIN_NUM);
        int source_pbin_match_idx = wrap_idx(target_bin_idx + di - 1, HIST_BIN_NUM);


        if(source_hist->getBinCountAt(source_bin_match_idx)==0 &&
           source_hist->getBinCountAt(source_nbin_match_idx)==0 &&
           source_hist->getBinCountAt(source_pbin_match_idx)==0 )
            continue;

        
        if(source_hist->getBinCountAt(source_bin_match_idx) > 0) {
            double s_bin_val = source_hist->getBinValueAt(source_bin_match_idx);
            double t_bin_val = target_hist->getBinValueAt(target_bin_idx);
            double multi_xy = s_bin_val * t_bin_val;
            cross_correlation_score += multi_xy;
            bin_pairs.push_back({source_bin_match_idx, target_bin_idx});

            double s_bin_mean = source_hist->getBinMeanAt(source_bin_match_idx);
            double t_bin_mean = target_hist->getBinMeanAt(target_bin_idx);
            double diff_theta = s_bin_mean - t_bin_mean;

            // handle shift angle confusion.
            if(diff_theta > 90)
                diff_theta -= 180;
            else if(diff_theta <= -90)
                diff_theta += 180;

            match_cnt ++;
            psi += diff_theta;            
        }
        else if(source_hist->getBinCountAt(source_nbin_match_idx) > 0) {
            double s_bin_val = source_hist->getBinValueAt(source_nbin_match_idx);
            double t_bin_val = target_hist->getBinValueAt(target_bin_idx);
            double multi_xy = s_bin_val * t_bin_val;
            cross_correlation_score += multi_xy;
            bin_pairs.push_back({source_nbin_match_idx, target_bin_idx});

            double s_bin_mean = source_hist->getBinMeanAt(source_nbin_match_idx);
            double t_bin_mean = target_hist->getBinMeanAt(target_bin_idx);
            double diff_theta = s_bin_mean - t_bin_mean;

            // handle shift angle confusion.
            if(diff_theta > 90)
                diff_theta -= 180;
            else if(diff_theta <= -90)
                diff_theta += 180;

            match_cnt ++;
            psi += diff_theta;              
        }
        else {
            double s_bin_val = source_hist->getBinValueAt(source_pbin_match_idx);
            double t_bin_val = target_hist->getBinValueAt(target_bin_idx);
            double multi_xy = s_bin_val * t_bin_val;
            cross_correlation_score += multi_xy;
            bin_pairs.push_back({source_pbin_match_idx, target_bin_idx});

            double s_bin_mean = source_hist->getBinMeanAt(source_pbin_match_idx);
            double t_bin_mean = target_hist->getBinMeanAt(target_bin_idx);
            double diff_theta = s_bin_mean - t_bin_mean;

            // handle shift angle confusion.
            if(diff_theta > 90)
                diff_theta -= 180;
            else if(diff_theta <= -90)
                diff_theta += 180;

            match_cnt ++;
            psi += diff_theta;              
        }
    }

    if(match_cnt > 0)
        psi /= match_cnt;

    return cross_correlation_score;
}

void PlanePairing::votingForConsistentPlanePairs(const double& psi,
    const std::vector<std::vector<int>>& s_plane_sets,
    const std::vector<std::vector<int>>& t_plane_sets) {
    match_idxes.clear();
    match_errors.clear();

    for(int i=0; i < s_plane_sets.size(); i++) {
        std::vector<int> s_plane_idxs, t_plane_idxs;
        s_plane_idxs = s_plane_sets[i];
        t_plane_idxs = t_plane_sets[i];

        std::sort(s_plane_idxs.begin(), s_plane_idxs.end());
        std::sort(t_plane_idxs.begin(), t_plane_idxs.end());

        // 1. coarse matching.      
        std::vector<std::pair<int,int>> potential_pairs;
        std::vector<double> weights;
        int pot_cnt = 0;
        for(int j=0; j < s_plane_idxs.size(); j++)
        {   
            int s_pidx = s_plane_idxs[j];
            for(int k=0; k  < t_plane_idxs.size(); k++)
            {
                int t_pidx = t_plane_idxs[k];
                double over_len = 0;
                if(twoPlaneSimilar(s_pidx, t_pidx, psi, over_len)) {
                    pot_cnt ++;
                    potential_pairs.push_back({s_pidx, t_pidx});
                    weights.push_back(over_len);
                }   
            }
        }

        // 2. voting consistent Distance difference to reject outliers in potential_pairs.
        std::vector<std::pair<int, int>> culled_pairs;
        // culled_pairs = votingConsistentDistanceDifference(potential_pairs);
        culled_pairs = votingConsistentDistanceDifferenceWithWeights(potential_pairs, weights);


        // 3. update match weights. 
        match_idxes.insert(match_idxes.end(), culled_pairs.begin(), culled_pairs.end());
        std::vector<float> cull_weights(weights.begin(), weights.end());

        match_errors.insert(match_errors.end(), cull_weights.begin(), cull_weights.end());


    }


    return;
}

void rotatePoint(POINT pt_in, POINT& pt_out, double psi_rad) {
    double cos_psi, sin_psi;
    cos_psi = std::cos(psi_rad);
    sin_psi = std::sin(psi_rad);

    pt_out.x = cos_psi*pt_in.x - sin_psi*pt_in.y;
    pt_out.y = sin_psi*pt_in.x + cos_psi*pt_in.y;
}

// first, check theta similarity, if d_theta < 2 deg.
// then check line projection overlap, 
//      use two rotated end points , project them onto source line, 
//      check if one of two points fallen in [xy] intervals of source end points.  
bool PlanePairing::twoPlaneSimilar(int s_pidx, int t_pidx, double psi, double& over_len) {
    bool similar_flag = false;

    // old implementation using viewpoint overlap.
    // int s_len, t_len;
    // int s_left = getPlaneLeftAt(*source_lines, s_pidx);
    // int s_right = getPlaneRightAt(*source_lines, s_pidx);
    // int t_left = getPlaneLeftAt(*target_lines, t_pidx);
    // int t_right = getPlaneRightAt(*target_lines, t_pidx);

    // s_len = s_right - s_left + 1;
    // t_len = t_right - t_left + 1;

    // t_left += target_idx_shift;
    // t_right += target_idx_shift;

    // int min_left, max_left, min_right, max_right;
    // min_left = std::min(s_left, t_left);
    // max_left = std::max(s_left, t_left);
    // min_right = std::min(s_right, t_right);
    // max_right = std::max(s_right, t_right);

    // if(max_left < min_right) {
    //  double overlap_ratio = (double)(min_right - max_left)/std::min(s_len, t_len);
    //  // cout << "overlap ratio: " << overlap_ratio << endl;
    //  if(overlap_ratio > PLANE_SIMILAR_RATIO)
    //      similar_flag = true;
    // }

    // new implementation using parallel projection overlap.

    // first, check theta similarity.
    double s_theta, t_theta;
    s_theta = (*source_lines)[s_pidx].theta;
    t_theta = (*target_lines)[t_pidx].theta;
    double d_theta = s_theta - t_theta;

    if(d_theta > M_PI)
        d_theta -= 2*M_PI;
    else if(d_theta < -M_PI)
        d_theta += 2*M_PI;
    


    double s_vdist, t_vdist;
    s_vdist = (*source_lines)[s_pidx].vdist;
    t_vdist = (*target_lines)[t_pidx].vdist;
    double d_vdist = s_vdist - t_vdist;

    if(std::fabs(d_vdist) > m_max_deltaDist) {
        return similar_flag;
    }

    if(std::fabs(d_theta) > M_PI/2) {
        if( s_vdist < 0.5 && t_vdist < 0.5) {
            // cout << "" << endl;
        } else {
            return similar_flag;            
        }
    }

    double theta_similarity = std::fabs(d_theta*RAD2DEG_RATIO - psi);


    if(theta_similarity > PLANE_SIMILAR_RATIO)
        return similar_flag;

    // then check line projection overlap.
    POINT s_p1, s_p2, t_rot_p1, t_rot_p2;

    // get source end point.
    s_p1 = (*source_lines)[s_pidx].p1;
    s_p2 = (*source_lines)[s_pidx].p2;

    // rotate two target end points, use psi
    double psi_rad = psi*DEG2RAD_RATIO;
    rotatePoint((*target_lines)[t_pidx].p1, t_rot_p1, psi_rad);
    rotatePoint((*target_lines)[t_pidx].p2, t_rot_p2, psi_rad);

    // find vertical projection point on source line.
    double sla, slb, slc;
    sla = (*source_lines)[s_pidx].a;
    slb = (*source_lines)[s_pidx].b;
    slc = (*source_lines)[s_pidx].c;
    POINT tp1_in_s, tp2_in_s;
    tp1_in_s = calcFootOfPerpendicular(sla, slb, slc, t_rot_p1);
    tp2_in_s = calcFootOfPerpendicular(sla, slb, slc, t_rot_p2);

    // check line overlap.
    over_len = 0;
    similar_flag = checkLineOverlap(s_p1, s_p2, tp1_in_s, tp2_in_s, over_len);




    return similar_flag;
}

POINT PlanePairing::calcFootOfPerpendicular(double la, double lb, double lc, POINT pt)
{
    POINT vpt;
    double denom = la*la + lb*lb;
    double lambda = la*pt.y - lb*pt.x;
    vpt.x = -(la*lc + lb*lambda) / denom;
    vpt.y = -(lb*lc - la*lambda) / denom; 
    return vpt;
}


///////////// OVLERLAP CASE /////////////////////////
//case 1:
//S: ------------------[-----]-----------------------
//T: ----------------------[-----]-------------------

//case 2:
//S: ------------------[-----]-----------------------
//T: --------------[-----]---------------------------

//case 3:
//S: ------------[-----------]-----------------------
//T: --------------[-----]---------------------------

//case 4:
//S: ------------[------]----------------------------
//T: -------- -[-----------]-------------------------

///////////// NON-OVLERLAP CASE /////////////////////////
//case 1:
//S: ------------------[-----]-----------------------
//T: --------[-----]---------------------------------

//case 2:
//S: ------------[-----]-----------------------------
//T: ----------------------[-----]-------------------
bool PlanePairing::checkLineOverlap(POINT s_p1, POINT s_p2, POINT ts_p1, POINT ts_p2, double& over_len) {
    bool flag = true;

    double s_max_x = std::max(s_p1.x, s_p2.x);
    double s_min_x = std::min(s_p1.x, s_p2.x);
    double s_max_y = std::max(s_p1.y, s_p2.y);
    double s_min_y = std::min(s_p1.y, s_p2.y);

    auto onSegmentSide = [=](POINT r1, POINT r2) -> int {

        if (r1.x > s_max_x && r2.x > s_max_x )
           return 1;
        else if(r1.x < s_min_x && r2.x < s_min_x)
            return -1;
        else if(r1.y > s_max_y && r2.y > s_max_y )
            return 2;
        else if(r1.y < s_min_y && r2.y < s_min_y)
            return -2;

        return 0;
    };

    int same_side_flag;
    same_side_flag = onSegmentSide(ts_p1, ts_p2);

    if(same_side_flag!=0) {
        return false;
    }

    // calculate overlap line segments.
    // normalize target line.
    double t_max_x = std::max(ts_p1.x, ts_p2.x);
    double t_min_x = std::min(ts_p1.x, ts_p2.x);
    double t_max_y = std::max(ts_p1.y, ts_p2.y);
    double t_min_y = std::min(ts_p1.y, ts_p2.y);

    // get overlap regions.
    double o_min_x, o_max_x, o_min_y, o_max_y;
    o_min_x = std::max(s_min_x, t_min_x);
    o_max_x = std::min(s_max_x, t_max_x);
    o_min_y = std::max(s_min_y, t_min_y);
    o_max_y = std::min(s_max_y, t_max_y);   

    double diff_o_x = o_max_x - o_min_x;
    double diff_o_y = o_max_y - o_min_y;

    over_len = std::sqrt(diff_o_x*diff_o_x + diff_o_y*diff_o_y);

    return flag;
}

std::vector<std::pair<int, int>> 
    PlanePairing::votingConsistentDistanceDifference(
        const std::vector<std::pair<int, int>>& potential_pairs) {

    if(potential_pairs.size()<=1)
        return potential_pairs;

    std::vector<double> dist_diff_vecs;
    dist_diff_vecs = computeDistDiffVecs(potential_pairs);



    // voting to get maximum consensus diff distance.
    // double noise_bound = 0.06;
    // std::vector<int> inlier_set = voting1DVec(dist_diff_vecs, noise_bound);
    std::vector<int> inlier_set = voting1DVec(dist_diff_vecs, VOTING_NOISE_BOUND);



    std::vector<std::pair<int, int>> consensus_pairs;
    for(int i=0; i < inlier_set.size(); i++) {
        int inlier_idx = inlier_set[i];
        consensus_pairs.push_back(potential_pairs[inlier_idx]);
    }
    return consensus_pairs; 
}

std::vector<std::pair<int, int>> 
    PlanePairing::votingConsistentDistanceDifferenceWithWeights(
        const std::vector<std::pair<int, int>>& potential_pairs,
        std::vector<double>& weights) {

    if(potential_pairs.size()<=1)
        return potential_pairs;

    std::vector<double> dist_diff_vecs;
    dist_diff_vecs = computeDistDiffVecs(potential_pairs);


    // voting to get maximum consensus diff distance.
    double noise_bound = 0.06;
    std::vector<int> inlier_set = voting1DVecWithWeights(dist_diff_vecs, weights, noise_bound);

    std::vector<double> consensus_weights;
    std::vector<std::pair<int, int>> consensus_pairs;
    for(int i=0; i < inlier_set.size(); i++) {
        int inlier_idx = inlier_set[i];
        consensus_pairs.push_back(potential_pairs[inlier_idx]);
        consensus_weights.push_back(weights[inlier_idx]);
    }
    weights = consensus_weights;
    return consensus_pairs; 
}


std::vector<double> PlanePairing::computeDistDiffVecs(const std::vector<std::pair<int,int>>& plane_pairs) {
    std::vector<double> diff_distances;
    for(int i=0; i < plane_pairs.size(); i++) {
        int s_pidx = plane_pairs[i].first;
        int t_pidx = plane_pairs[i].second;
        double s_dist = (*source_lines)[s_pidx].vdist;
        double t_dist = (*target_lines)[t_pidx].vdist;
        double delta_d = std::fabs(s_dist - t_dist);
        diff_distances.push_back(delta_d);
    }
    return diff_distances;
}



std::vector<int> PlanePairing::voting1DVecWithWeights(const std::vector<double>& vec_1d,
    const std::vector<double> &weights, 
    const double& noise_bound) {
    int N = vec_1d.size();
    
    std::vector<double> data_interval_edges;
    for(int i=0; i < N; i++) {
        double m_minus_n = vec_1d[i] - noise_bound;
        double m_plus_n = vec_1d[i] + noise_bound;
        data_interval_edges.push_back(m_minus_n);
        data_interval_edges.push_back(m_plus_n);
    }
    std::sort(data_interval_edges.begin(), data_interval_edges.end());

    // get middle value of interval.
    std::vector<double> data_interval_mids(2*N-1);
    for(int i=0; i < 2*N-1; i++) {
        double mid_val = (data_interval_edges[i] + data_interval_edges[i+1])/2;
        data_interval_mids[i] = mid_val;
    }

    // voting mid values.
    std::vector<std::vector<int>> voting_index_sets(2*N-1);
    std::vector<double> voting_counts(2*N-1, 0);
    double max_consensus_cnt = 0;
    int max_consensus_idx = -1;

    for(int i=0; i < data_interval_mids.size(); i++) {
        double mid_val = data_interval_mids[i];
        for(int j=0; j < N; j++) {  
 
            double m_minus_n = vec_1d[j] - noise_bound;
            double m_plus_n = vec_1d[j] + noise_bound;
            if(mid_val >= m_minus_n && mid_val < m_plus_n) {
                voting_index_sets[i].push_back(j);
                voting_counts[i] += weights[j];
                if(max_consensus_cnt < voting_counts[i]) {
                    max_consensus_cnt = voting_counts[i];
                    max_consensus_idx = i;
                }
            }
        }
    }


    std::vector<int> consensus_set;
    if(max_consensus_idx>=0) {
        consensus_set = voting_index_sets[max_consensus_idx];
    }


    return consensus_set;
}
