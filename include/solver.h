//
// Created by bzdfzfer on 2022/8/26.
//
#ifndef CFM_SOLVER_H
#define CFM_SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common.h"


#include <vector>
#include <random>
#include <iostream>
using std::cos;
using std::sin;
using std::cout;
using std::endl;



class Solver
{
public:
    Solver() {}
    ~Solver() {}
    virtual void preprocessing() = 0;
    virtual void calTranslation() = 0;
    virtual void calRotation() = 0;

protected:
    std::vector<std::vector<float>> nS, nT;
    std::vector<float> distS, distT;
    std::vector<float> w;
};


class Solver2d_polar : public Solver
{
protected:
    int dim = 2;
    std::vector<std::vector<float>> XY;
    std::vector<float> vecb;
    Eigen::Matrix<float, 2, 1> matX;
    std::vector<float> deltaTheta;
    float deltaPhi;

    Eigen::Matrix<float, 2, 1> last_matX;
    float last_deltaPhi;

    std::vector<int> m_inliers_set;
    float ANGULAR_NOISE_BOUND = 2.5*DEG2RAD_RATIO;

    float eigenThres[2] = {0.1, 0.1};

    bool m_degenerated_flag = false;

public:

    void setAngularNoiseBound(float nb) { ANGULAR_NOISE_BOUND = nb; }

    void setDegenerateEigenThreshold(float eig_thres1, float eig_thres2) { 
        eigenThres[0] = eig_thres1; 
        eigenThres[1] = eig_thres2; 
    }

    void run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& s_norm,
             const pcl::PointCloud<pcl::PointXYZI>::Ptr& t_norm,
             const std::vector<std::pair<int, int>> match_idxes);


    void preprocessing() override;
    void calTranslation() override;
    void calRotation()  override;


    Eigen::Vector2f getTranslation() { return matX; }
    float getHeadingAngle() { return deltaPhi; }

    Eigen::Matrix2f getRotationMatrix() {
        Eigen::Matrix2f res;
        res(0,0) = cos(deltaPhi);   res(0,1) = -sin(deltaPhi);
        res(1,0) = -res(0,1);       res(1,1) = res(0,0);
        return res;
    }

    Eigen::Matrix2f getRotationMatrix(float phi) {
        Eigen::Matrix2f res;
        res(0,0) = cos(phi);   res(0,1) = -sin(phi);
        res(1,0) = -res(0,1);       res(1,1) = res(0,0);       
        return res; 
    }

    Eigen::Matrix3f getTransform() {
        Eigen::Matrix3f res = Eigen::Matrix3f::Identity();
        res.block<2,2>(0,0) = getRotationMatrix();
        res.block<2,1>(0,2) = matX;
        return res;
    }

    Eigen::Matrix3f getTransform(Eigen::Vector2f trans, float rot_ang) {
        Eigen::Matrix3f res = Eigen::Matrix3f::Identity();
        res.block<2,2>(0,0) = getRotationMatrix(rot_ang);
        res.block<2,1>(0,2) = trans;
        return res;        
    }

    void setWeights(const std::vector<float>& weights) {
        w = weights;
    }    
};

#endif