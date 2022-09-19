//
// Created by bzdfzfer on 2022/8/26.
//

#include "solver.h"

void Solver2d_polar::run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& s_norm,
             const pcl::PointCloud<pcl::PointXYZI>::Ptr& t_norm,
             const std::vector<std::pair<int, int>> match_idxes) {

    nS.clear();     nT.clear();
    distS.clear();  distT.clear();

    for(int i=0; i < match_idxes.size(); i++) {
        int s_idx = match_idxes[i].first;
        int t_idx = match_idxes[i].second;
        nS.push_back({s_norm->points[s_idx].x, s_norm->points[s_idx].y});
        nT.push_back({t_norm->points[t_idx].x, t_norm->points[t_idx].y});
        distS.push_back(s_norm->points[s_idx].intensity);
        distT.push_back(t_norm->points[t_idx].intensity);
    }

    preprocessing();
    calRotation();
    // cullingNoisyEdgesWithdPhi();
    calTranslation();

    last_deltaPhi = deltaPhi;
    last_matX = matX;
}

void Solver2d_polar::preprocessing()  {
    int pairNum = nS.size();
    deltaTheta.clear();
    XY.clear();
    vecb.clear();
    m_inliers_set.clear();
    for (int i = 0; i < pairNum; ++i) {
        float theta_s = std::atan2(nS[i][1], nS[i][0]);
        float theta_t = std::atan2(nT[i][1], nT[i][0]);
        float d_theta =  theta_s - theta_t;
        // if(d_theta > M_PI/2) {
        //     d_theta -= M_PI/2;
        // } else if(d_theta <= -M_PI/2) {
        //     d_theta += M_PI/2;
        // }
        if(d_theta > M_PI) {
            d_theta -= 2*M_PI;
        } else if(d_theta < -M_PI) {
            d_theta += 2*M_PI;
        }

        deltaTheta.push_back(d_theta);

        float dd = (distS[i] - distT[i]);

        XY.push_back({nS[i][0], nS[i][1]});
        vecb.push_back(dd);

        // cout << "distS: " << distS[i] << ", distT: " << distT[i] << endl; 
        
        // cout << "Nxy_s--delta_d: (" << nS[i][0] << ", " << nS[i][1] << ") -- " << dd << endl;

        // if (distS[i] > distT[i]) {
        //     XY.push_back({nS[i][0], nS[i][1]});
        // }
        // else {
        //     XY.push_back({-nS[i][0], -nS[i][1]});
        // }
    }
}

void Solver2d_polar::calTranslation()  {


    int pairNum = XY.size();
    if(pairNum == 0) {
        return;
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matA(pairNum, 2);
    Eigen::Matrix<float, 2, Eigen::Dynamic> matAt(2, pairNum);
//        auto mat = vec.asDiagonal();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matW = Eigen::MatrixXf::Identity(pairNum, pairNum);
    Eigen::Matrix<float, 2, 2> matAtWA;
    Eigen::VectorXf matB(pairNum);
    Eigen::Matrix<float, 2, 1> matAtWB;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matAW(pairNum, 2);
    Eigen::VectorXf matBW(pairNum);

    float heading_x = std::cos(deltaPhi/2);
    float heading_y = std::sin(deltaPhi/2);

    // float heading_x = std::cos(deltaPhi);
    // float heading_y = std::sin(deltaPhi);

    float w_sum = 0.0;
    for (int i = 0; i < pairNum; ++i){
        float wt = w[i];

        // wt = w[i]*w[i];
        // float cos_hd_norm = std::fabs( heading_x*XY[i][0]  + heading_y*XY[i][1] );
        // wt *= cos_hd_norm;

        matW(i, i) = wt;
        w_sum += wt;
        matB(i) = vecb[i];
        matBW(i) = vecb[i]*wt;


        for (int j = 0; j < 2; ++j) {
            matA(i, j) = XY[i][j];
            matAW(i, j) = XY[i][j]*wt;
        }
    }
//        matW = w.asDiagonal();

    matAt = matA.transpose();
    matAtWA = matAt * matW * matA;
    matAtWB = matAt * matW * matB;

    // int vN = m_inliers_set.size();
    // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matA(vN, 2);
    // Eigen::Matrix<float, 2, Eigen::Dynamic> matAt(2, vN);
    // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matW = Eigen::MatrixXf::Identity(vN, vN);
    // Eigen::Matrix<float, 2, 2> matAtWA;
    // Eigen::Matrix<float, 2, 1> matAtWB;
    // Eigen::VectorXf matB(vN);
    // for(int i=0; i < vN; i++) {
    //     int idx = m_inliers_set[i];
    //     matW(i,i) = w[idx];
    //     matB(i) = vecb[idx];
    //     matA(i, 0) = XY[idx][0];
    //     matA(i, 1) = XY[idx][1];
    // } 

    Eigen::Matrix<float, 1, 2> matE;
    Eigen::Matrix<float, 2, 2> matV;
    Eigen::Matrix<float, 2, 2> matV2;
    Eigen::Matrix<float, 2, 2> matP;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 2, 2> > esolver(matAtWA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;
    m_degenerated_flag = false;
    bool degenFlag[] = {false, false};
    for(int i=0; i <2; i++) {
        if(std::fabs(matE(0,i)) < eigenThres[i])
        {
            m_degenerated_flag = true;
            // cout << "matE[" << 0 << "]: " << matE(0, 0) << endl;
            // cout << "matE[" << 1 << "]: " << matE(0, 1) << endl;

        }
        else {
            break;
        }
    }

    if(m_degenerated_flag) {
        fprintf(stderr, "degenerated scene happened ..... \n");
        // matX = matAW.transpose() * matBW / w_sum / w_sum;

        fprintf(stderr, "deltaPhi: %f \n", deltaPhi);

        int pos_cnt = 0;
        int neg_cnt = 0;
        int zero_cnt = 0;
        float pos_weighted_sum = 0, pos_weights = 0, neg_weighted_sum = 0, neg_weights = 0;

        for(int i=0; i < pairNum; i++) {
            fprintf(stderr, "equation [%d]: (a1,a2)=(%f, %f), b= %f \n ", i,
                matA(i, 0), matA(i, 1), matB(i));

            float cos_hd_norm = heading_x*matA(i, 0)  + heading_y*matA(i, 1);
            float cos_times_b = cos_hd_norm*matB(i);
            if(cos_times_b > 1e-3) {
                pos_cnt ++;
                pos_weighted_sum += matB(i)/cos_hd_norm*w[i];
                pos_weights += w[i];
            } else if(cos_times_b < -1e-3) {
                neg_cnt ++;
                neg_weighted_sum += matB(i)/cos_hd_norm*w[i];
                neg_weights += w[i];                    
            } else { // zero happens, distance is 0, or plane parallel to robot motion.
                zero_cnt ++;
            }
        }

        if(pos_cnt > neg_cnt && neg_cnt >= zero_cnt) {
            float heading_dist = pos_weighted_sum/pos_weights;
            matX(0) = heading_x * heading_dist;
            matX(1) = heading_y * heading_dist;
            fprintf(stderr, "positive distance is : %f \n", heading_dist);
        } else if(neg_cnt > pos_cnt && neg_cnt >= zero_cnt){
            float heading_dist = neg_weighted_sum/neg_weights;
            matX(0) = heading_x * heading_dist;
            matX(1) = heading_y * heading_dist;
            fprintf(stderr, "negative distance is : %f \n", heading_dist);
        } else {
            matX = last_matX;
        }


        return;

        // failed (this solution is random).
        // x = At*W * (A*At*W)-1 b
        // -----------------------------------------------------------------------
        // Eigen::VectorXf matXtmp(pairNum);
        // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matAAtW(pairNum, pairNum);
        // matAAtW = matA * matAt* matW;
        // matXtmp = matAAtW.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matB);
        // matX = matAt * matW * matXtmp;

        // compare with last matX.
        // fprintf(stderr, "cur matX: (%f, %f), last matX: (%f, %f) \n", 
        //     matX(0), matX(1), last_matX(0), last_matX(1));
        // -----------------------------------------------------------------------

        return;
    }



    // matX = (matA.transpose() * matA).ldlt().solve(matA.transpose() * matB);
    // matX = matAtWA.ldlt().solve(matAtWB);
    // matX = matA.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matB);
    matX = matAW.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matBW);
    // matX = matAtWA.colPivHouseholderQr().solve(matAtWB);
    // matX = matAtWA.FulPivHouseholderQr().solve(matAtWB);


    // check motion-normal orgothonal condition.
    int x_cnt =0, y_cnt = 0, reg_cnt = 0;
    float x_weights=0, y_weights=0;
    std::vector<int> x_norm_idxs;
    for(int i=0; i < pairNum; i++ ) {
        float cos_xdir_norm = std::fabs(matA(i,0));
        if(cos_xdir_norm > 0.9) {
            x_cnt ++;
            x_weights += w[i];
            x_norm_idxs.push_back(i);
        }
        else {
            y_cnt ++;
            y_weights += w[i];
        }
    }


    // fprintf(stderr, "x dir norm count: %d, y cnt: %d, x_weights: %f, y_weights: %f, reg_cnt: %d \n",
    //     x_cnt,  y_cnt, x_weights, y_weights, reg_cnt);

    Eigen::Vector2f norm_X = matX/matX.norm();
    Eigen::Vector2f heading_Phi;
    heading_Phi << heading_x, heading_y;
    float cos_X_Phi = std::fabs(norm_X.dot(heading_Phi));

    // fprintf(stderr, "cos matX and deltaPhi: %f \n", cos_X_Phi);

    if(cos_X_Phi < 0.95 && x_cnt > y_cnt  && x_weights >= 4.0*y_weights && y_weights > 0.1 && y_weights < 3.4 ) {

        fprintf(stderr, "degenerated in vertical constraints ....... \n");

        int pos_cnt = 0;
        int neg_cnt = 0;
        int zero_cnt = 0;
        float pos_weighted_sum = 0, pos_weights = 0, neg_weighted_sum = 0, neg_weights = 0;

        for(int i=0; i < x_norm_idxs.size(); i++) {
            int a_idx = x_norm_idxs[i];
            float a1 = matA(a_idx, 0);
            float a2 = matA(a_idx, 1);
            float b1 = matB(a_idx);
            fprintf(stderr, "equation [%d]: (a1,a2)=(%f, %f), b= %f \n ", i,
               a1 , a2, b1);

            float cos_hd_norm = heading_x*a1  + heading_y*a2;
            float cos_times_b = cos_hd_norm*b1;
            if(cos_times_b > 1e-6) {
                pos_cnt ++;
                pos_weighted_sum += b1/cos_hd_norm*w[a_idx];
                pos_weights += w[a_idx];
            } else if(cos_times_b < -1e-6) {
                neg_cnt ++;
                neg_weighted_sum += b1/cos_hd_norm*w[a_idx];
                neg_weights += w[a_idx];                    
            } else { // zero happens, distance is 0, or plane parallel to robot motion.
                zero_cnt ++;
            }
        }

        if(pos_cnt > neg_cnt ) {
            float heading_dist = pos_weighted_sum/pos_weights;
            matX(0) = heading_x * heading_dist;
            matX(1) = heading_y * heading_dist;
            fprintf(stderr, "positive distance is : %f \n", heading_dist);
        } else if(neg_cnt > pos_cnt){
            float heading_dist = neg_weighted_sum/neg_weights;
            matX(0) = heading_x * heading_dist;
            matX(1) = heading_y * heading_dist;
            fprintf(stderr, "negative distance is : %f \n", heading_dist);
        }
    }

    // check motion continuity.
    // Eigen::Matrix3f cur_DT = getTransform();
    // Eigen::Matrix3f last_DT = getTransform(last_matX, last_deltaPhi);
    // Eigen::Matrix3f diff_DT = last_DT.inverse() * cur_DT;
    // Eigen::Vector2f diff_trans = diff_DT.block<1,2>(0,2);
    // fprintf(stderr, "difference trans vector: %f , %f \n", diff_trans(0), diff_trans(1) );
    // float diff_norm = diff_trans.norm();
    // if(diff_norm > 0.35*last_matX.norm() && last_matX.norm() > 0.001) {
    //     fprintf(stderr, "motion jump happens: %f \n", diff_norm);
    //     matX = last_matX;
    // }

    // find the degenerated cases.
    // Eigen::Matrix<float, 1, 2> matE;
    // Eigen::Matrix<float, 2, 2> matV;
    // Eigen::Matrix<float, 2, 2> matV2;
    // Eigen::Matrix<float, 2, 2> matP;
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 2, 2> > esolver(matAtWA);
    // matE = esolver.eigenvalues().real();
    // matV = esolver.eigenvectors().real();
    // matV2 = matV;
    // bool isDegenerate = false;
    // float eigenThres[] = {0.1, 0.1};
    // bool degenFlag[] = {false, false};
    // for(int i=0; i <2; i++) {
    //     if(std::fabs(matE(0,i)) < eigenThres[i])
    //     {
    //         isDegenerate = true;
    //         cout << "matE[" << 0 << "]: " << matE(0, 0) << endl;
    //         cout << "matE[" << 1 << "]: " << matE(0, 1) << endl;

    //     }
    //     else {
    //         break;
    //     }
    // }

    // if(isDegenerate) {
    //     matX = Eigen::Vector2f::Zero();
    // }
}

void Solver2d_polar::calRotation()  {

    // m_inliers_set = voting1DVec(deltaTheta, ANGULAR_NOISE_BOUND);

    // int n = m_inliers_set.size();
    // float sum = 0;
    // float sum_w = 0;
    // for (int i = 0; i < n; ++i) {
    //     sum += deltaTheta[m_inliers_set[i]] * w[m_inliers_set[i]];
    //     sum_w += w[m_inliers_set[i]];
    // }
    // deltaPhi = sum / sum_w;


    int n = deltaTheta.size();
    float sum = 0;
    float sum_w = 0;
    for (int i = 0; i < n; ++i) {
        sum += deltaTheta[i] * w[i];
        sum_w += w[i];
    }
    deltaPhi = sum / sum_w;

    // int n = deltaTheta.size();
    // std::vector<float> s_deltaTheta;
    // s_deltaTheta = deltaTheta;
    // std::sort(s_deltaTheta.begin(), s_deltaTheta.end());
    // deltaPhi = s_deltaTheta[n/2];
}

