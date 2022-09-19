//
// Created by bzdfzfer on 2022/8/26.
//

#include "line_feature.h"
#include "sys/time.h"
#include <limits>
#include <iostream>

namespace line_feature
{
// constructor
LineFeature::LineFeature()
{

}    

// destructor
LineFeature::~LineFeature()
{

}    

//set paramters
void LineFeature::set_angle_increment(double angle_increment)
{
    params_.angle_increment = angle_increment;
}

void LineFeature::set_angle_start(double angle_start)
{
    params_.angle_start = angle_start;
}

void LineFeature::set_least_threshold(double least_thresh)
{
    params_.least_thresh = least_thresh;
}

void LineFeature::set_min_line_length(double min_line_length)
{
    params_.min_line_length = min_line_length;
}

void LineFeature::set_predict_distance(double predict_distance)
{
    params_.predict_distance = predict_distance;
}

void LineFeature::set_min_line_points(unsigned int min_line_points)
{
    params_.min_line_points = min_line_points;
}

void LineFeature::set_seed_line_points(unsigned int seed_line_points)
{
    params_.seed_line_points = seed_line_points;
}

void LineFeature::set_laser_points_num(unsigned int laser_pts_num) {
    params_.laser_points_num = laser_pts_num;
}


void LineFeature::set_pts_missing_tolerance(unsigned int missing_num) {
    params_.pts_missing_tolerance = missing_num;
}

void LineFeature::set_max_pts_gap(double max_pts_gap) {
    params_.max_pts_gap = max_pts_gap;
}
    

void LineFeature::setCosSinData(const std::vector<double>& bearings,
                                const std::vector<double>& cos_value,
                                const std::vector<double>& sin_value,
                                const std::vector<unsigned int>& index)
{
    cs_data_.index = index;
    cs_data_.cos_value = cos_value;
    cs_data_.sin_value = sin_value;
    cs_data_.bearings = bearings;
}

void LineFeature::setCSDataIdx(const std::vector<unsigned int>& index) {
    cs_data_.index = index;
}



// set input data.
void LineFeature::setRangeData(const std::vector<double>& ranges)
{
    range_data_.ranges = ranges;
    range_data_.xs.clear();
    range_data_.ys.clear();
    m_valid_index.clear();
    int i=0;
    for (std::vector<unsigned int>::const_iterator cit = cs_data_.index.begin();
         cit != cs_data_.index.end(); ++cit)
    {
        if(std::isnan(ranges[*cit]) || ranges[*cit] > m_max_range ) {
            // fprintf(stderr, "[LF-91] `````` invalid laser points occured at: %d\n", i);
        }
        else {
            m_valid_index.push_back(i);
        }
        i++;
        range_data_.xs.push_back(cs_data_.cos_value[*cit] * ranges[*cit]);
        range_data_.ys.push_back(cs_data_.sin_value[*cit] * ranges[*cit]);
    }
}

void LineFeature::setRangeData(const std::vector<double>& xs, const std::vector<double>& ys) {

    range_data_.xs = xs;
    range_data_.ys = ys;
    range_data_.ranges.clear();
    m_valid_index.clear();
    for(int i =0; i < xs.size(); i++) {
        double range = sqrt(xs[i]*xs[i] + ys[i]*ys[i]);
        if(std::isnan(xs[i]) || std::isnan(ys[i]) || range > m_max_range)
        {
            // fprintf(stderr, "[LF-106]+++++++ invalid points occured at idx: %d\n", i);
        }
        else {
            m_valid_index.push_back(i);
        }
        // TODO: handle invalid points.
        range_data_.ranges.push_back(range);
    }
}

least LineFeature::leastsquare(int start,int end,int firstfit)
{
    double w1 = 0, w2 = 0, w3 = 0;
    least temp;
    double n = end - start + 1;

    
    //firstfit = true;
    int k_start = 0;
    int k_end = 0;

    if(firstfit == 1)
    {
        mid1 = 0;
        mid2 = 0;
        mid3 = 0;
        mid4 = 0;
        mid5 = 0;
        int i = 0;
        int k = 0;
        for(i = start;i <= end;i++)
        {
            k = m_valid_index[i];
            mid1+=range_data_.xs[k];
            mid2+=range_data_.ys[k];
            mid3+=range_data_.xs[k]*range_data_.xs[k];
            mid4+=range_data_.ys[k]*range_data_.ys[k];
            mid5+=range_data_.xs[k]*range_data_.ys[k];
        }                
    }
    else
    {
        if(firstfit == 2)
        {
            k_end = m_valid_index[end];
            mid1+=range_data_.xs[k_end];
            mid2+=range_data_.ys[k_end];
            mid3+=range_data_.xs[k_end]*range_data_.xs[k_end];
            mid4+=range_data_.ys[k_end]*range_data_.ys[k_end];
            mid5+=range_data_.xs[k_end]*range_data_.ys[k_end];                    
        }
        else
        {
            k_start = m_valid_index[start];
            mid1+=range_data_.xs[k_start];
            mid2+=range_data_.ys[k_start];
            mid3+=range_data_.xs[k_start]*range_data_.xs[k_start];
            mid4+=range_data_.ys[k_start]*range_data_.ys[k_start];
            mid5+=range_data_.xs[k_start]*range_data_.ys[k_start];
        }
    }

    w1 = n*mid5-mid1*mid2;
    w2 = mid2*mid2-n*mid4-mid1*mid1+n*mid3;
    w3 = mid1*mid2-n*mid5;

    // if(w1==0)
    // {
    //     temp.a = -1;
    //     temp.b = 0;
    //     temp.c = mid1/n;
    // }
    // else
    // {
    //     temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
    //     temp.b = -1;
    //     temp.c = (mid2-temp.a*mid1)/n;
    // }

    double det_x = n*mid3 - mid1*mid1;
    double det_y = n*mid4 - mid2*mid2;

    if(det_x > det_y) { // using : y=mx + b.
        if(w1 == 0)
        {
            temp.a = -1;
            temp.b = 0;
            temp.c = mid1/n;    
        } else {
            temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
            temp.b = -1;
            temp.c = (mid2-temp.a*mid1)/n;                
        }        
    } else { // det_x >= det_y --: using x=ky+b.
        if(w1 == 0)
        {
            temp.a = 0;
            temp.b = -1;
            temp.c = mid2/n;    
            if(std::isnan(temp.c)) {
                fprintf(stderr, "-- invalid lc ---- mid2: %f, n: %f, start: %d, end: %d, firstfit: %d \n", 
                    mid2, n, start, end, firstfit);
            }
        } else {
            temp.a = -1;
            temp.b = (w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
            temp.c = (mid1-temp.b*mid2)/n;                
        }


    }
    
    return temp;
}

bool LineFeature::detectline(const int start,const int num)
{

    bool flag = false;
    //定义点到直线的垂直误差
    double error1 = 0;
    //定义下一点到预测位置的误差
    double error2 = 0;
    int i = 0;
    int k = 0;
    //预测下一点位置
    POINT m_pn;
    m_pn.x = 0;
    m_pn.y = 0;
    //下一点，y = kp*x;
    double kp = 0;
    double theta = 0;

    for(i = start;i < start+num;i++)
    {
        k = m_valid_index[i];

        // 10 degrees. 2.5*10 = 25, 5 degs, 12.5 pts.
        double r1 = range_data_.ranges[k];
        if(i>=1 && 
            (k - m_valid_index[i-1]>params_.pts_missing_tolerance ||
             std::fabs(r1 - range_data_.ranges[m_valid_index[i-1]]) > params_.max_pts_gap) )
        {
            flag = true;
            break;
        } 

        //到直线的垂直距离
        error1 = fabs((m_least.a*range_data_.xs[k]+m_least.b*range_data_.ys[k]+m_least.c))/sqrt(m_least.b*m_least.b+m_least.a*m_least.a);
        if(error1 > params_.least_thresh)
        {
            flag = true;
            break;
        }

        // theta = params_.angle_increment*k + params_.angle_start;

        // double cos_theta = cos(theta);
        // double sin_theta = sin(theta);
        double cos_theta = cs_data_.cos_value[k];
        double sin_theta = cs_data_.sin_value[k];

        double den_acos_bsin = m_least.a*cos_theta + m_least.b*sin_theta;
        m_pn.x = -(m_least.c*cos_theta)/den_acos_bsin;
        m_pn.y = -(m_least.c*sin_theta)/den_acos_bsin;

        //计算到预测点之间的误差
        error2 = distance_point(range_data_.xs[k],range_data_.ys[k],m_pn.x,m_pn.y);
        
        if(error2 > params_.predict_distance)
        {
            flag = true;
            break;
        }
    }
    if(flag)
    {
        return false;
    }
    else
    {
        return true;
    }
}

//检测完整的直线段
int LineFeature::detectfulline(const int start)
{
    line m_temp;

    bool flag1 = true;
    bool flag2 = true;
    int n1 = 0;
    int n2 = 0;
    double a = 0;
    double b = 0;
    double c = 0;

    a = m_least.a;
    b = m_least.b;
    c = m_least.c;

    n2 = start + params_.seed_line_points;

    least m_result;
    m_result.a = 0;
    m_result.b = 0;
    m_result.c = 0;
    //向前生长
    while(flag2)
    {
        unsigned int idx = m_valid_index[n2];
        if(n2 >0 && idx - m_valid_index[n2-1] > params_.pts_missing_tolerance) {
            break;
        }

        // if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(1+a*a)))<params_.least_thresh)
        if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(b*b+a*a)))<params_.least_thresh)
        {
            m_least = leastsquare(start,n2,2);
            if(n2 < (m_valid_index.size() - 1))
            {
                n2 = n2 + 1;
                a = m_least.a;
                b = m_least.b;
                c = m_least.c;
            }
            else
            {
                flag2 = false;
            }
        }
        else
        {
            flag2 = false;
        }
    }

    if(n2 < m_valid_index.size()-1) {
        n2 = n2-1;
    }

    //向后回溯
    n1 = start - 1;
    if(n1 < 0)
    {
        flag1 = false;
    }
    while(flag1)
    {       
        unsigned int idx = m_valid_index[n1];
        if(n1 < range_data_.xs.size()-1 && m_valid_index[n1+1] - idx > params_.pts_missing_tolerance) {
            break;
        }

        // if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(1+a*a)))<params_.least_thresh)
        if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(b*b+a*a)))<params_.least_thresh)            
        {
            m_least = leastsquare(n1,n2,3);
            if(n1>0)
            {
                n1 = n1 - 1;
                a = m_least.a;
                b = m_least.b;
                c = m_least.c;
            }
            else
            {
                flag1 = false;
            }
        }
        else
        {
            flag1 = false;
        }
    }
    n1 = n1+1;

    m_temp.left = n1;
    m_temp.right = n2;
    //此处是统一再做一次拟合，可能以一定步长进行拟合搜索的时候，需要这样完整的拟合过程，此时不需要
    m_result = leastsquare(n1,n2,1);
    m_temp.a = m_result.a;
    m_temp.b = m_result.b;
    m_temp.c = m_result.c;


    // int left_fv_idx = m_valid_index[n1];
    // int right_fv_idx = m_valid_index[n2];
    // fprintf(stderr, "---- full line idx: (%d, %d) \n",
    //     left_fv_idx, right_fv_idx);
    // fprintf(stderr, "---- grown line param: %f, %f, %f \n",
    //     m_result.a, m_result.b, m_result.c);

    // fprintf(stderr, "---- min line points: %d \n", params_.min_line_points);

    if((m_valid_index[n2] - m_valid_index[n1]+1)>=params_.min_line_points)
    {
        // fprintf(stderr, " ---- (to be added) full line idx: (%d, %d) \n",
        //     left_fv_idx, right_fv_idx);         
        if(delete_short_line(m_valid_index[n1],m_valid_index[n2]))
        {
            // fprintf(stderr, "added line ---- full line idx: (%d, %d) \n",
            //     left_fv_idx, right_fv_idx);            
            m_line.push_back(m_temp);
        }
        return n2;
    }
    else
    {
        return start;
    }
}

void LineFeature::cleanline()
{
    if(m_line.size() < 2)
    {
        return;
    }

    int m = 0;
    int n = 0;
    int m_iter = 0;
    double error1 = 0;
    double error2 = 0;
    int line_temp = 0;
    least temp_least;
    temp_least.a = 0;
    temp_least.b = 0;
    temp_least.c = 0;

    double theta_one_ = 0;
    double theta_two_ = 0;
    double theta_d_ = 0;
    std::size_t q = 0,p = 0;

    for(p = 0; p < m_line.size() - 1; p++)
    {
        m = m_line[p].right;
        for(q = p+1;q < m_line.size();q++)
        {
            n = m_line[q].left;
            if(m >= n)
            {
                theta_one_ = atan(m_line[p].a);
                theta_two_ = atan(m_line[q].a);

                theta_d_ = fabs(theta_one_ - theta_two_);

                if((theta_d_<0.1)||(theta_d_>(PI - 0.1)))
                {
                    int _left = std::min(m_line[p].left,m_line[q].left);

                    least m_temp = leastsquare(_left,m_line[q].right,1);

                    m_line[p].a = m_temp.a;
                    m_line[p].b = m_temp.b;
                    m_line[p].c = m_temp.c;

                    m_line[p].left = _left;
                    m_line[p].right = m_line[q].right;

                    m_line.erase(m_line.begin()+q);

                    m = m_line[p].right;
                    q = q - 1;

                }


            }
        }
    }

    int k;
    //处理有相互链接关系的线段
    for(p = 0; p < (m_line.size() - 1); p++)
    {
        q = p+1;
        m = m_line[p].right;
        n = m_line[q].left;
        if(m >= n)
        {
            for(m_iter = n+params_.seed_line_points;m_iter <= m;m_iter++)
            {
                line_temp = m_iter;
                k = m_valid_index[m_iter];
                // error1 = fabs(((m_line[p].a)*range_data_.xs[k]+(m_line[p].b)*range_data_.ys[k]+m_line[p].c))/sqrt((1+(m_line[p].a)*(m_line[p].a)));
                // error2 = fabs(((m_line[q].a)*range_data_.xs[k]+(m_line[q].b)*range_data_.ys[k]+m_line[q].c))/sqrt((1+(m_line[q].a)*(m_line[q].a)));
                error1 = fabs(((m_line[p].a)*range_data_.xs[k]+(m_line[p].b)*range_data_.ys[k]+m_line[p].c))/sqrt((m_line[p].b*m_line[p].b+(m_line[p].a)*(m_line[p].a)));
                error2 = fabs(((m_line[q].a)*range_data_.xs[k]+(m_line[q].b)*range_data_.ys[k]+m_line[q].c))/sqrt((m_line[q].b*m_line[q].b+(m_line[q].a)*(m_line[q].a)));

                if(error1 > error2)
                {
                    break;
                }
            }
            m_line[p].right = m_iter-1;
            temp_least = leastsquare(m_line[p].left,m_line[p].right,1);
            m_line[p].a = temp_least.a;
            m_line[p].b = temp_least.b;
            m_line[p].c = temp_least.c;

            m_line[q].left = m_iter;
            temp_least = leastsquare(m_line[q].left,m_line[q].right,1);
            m_line[q].a = temp_least.a;
            m_line[q].b = temp_least.b;
            m_line[q].c = temp_least.c;
        }
    }
}

bool LineFeature::delete_short_line(const int n1,const int n2)
{
    double line_len = distance_point(range_data_.xs[n1],range_data_.ys[n1],range_data_.xs[n2],range_data_.ys[n2]);
    if(line_len<params_.min_line_length)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool LineFeature::delete_shadow_line(const line& l_t) {
    double dist = std::fabs(l_t.c/std::sqrt(l_t.b*l_t.b + l_t.a*l_t.a));
    if(dist < 0.2)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void LineFeature::generate(std::vector<gline>& temp_line2)
{
    gline line_temp;
    std::vector<gline> output;
    POINT endpoint1;
    POINT endpoint2;
    int m = 0,n = 0;
    double k1 = 0,k2 = 0;
    for(int i = 0;i < m_line.size();i++)
    {   
        m = m_valid_index[m_line[i].left];
        n = m_valid_index[m_line[i].right];

        m_line[i].left = m;
        m_line[i].right = n;


        calcVarianceOfFittedLine(m_line[i], m, n);



        double l_a = m_line[i].a;
        double l_b = m_line[i].b;
        double l_c = m_line[i].c;
        // double l_aa = l_a*l_a;
        // double l_ab = l_a*l_b;
        // double l_ac = l_a*l_c;
        // double l_bb = l_b*l_b;
        // double l_bc = l_b*l_c;
        // double l_aa_p_bb = l_aa + l_bb;
        // endpoint1.x = (l_bb*range_data_.xs[m] - l_ab*range_data_.ys[m] - l_ac)/l_aa_p_bb;
        // endpoint1.y = (l_aa*range_data_.ys[m] - l_ab*range_data_.xs[m] - l_bc)/l_aa_p_bb;

        double cos_theta1 = cs_data_.cos_value[m];
        double sin_theta1 = cs_data_.sin_value[m];
        double cos_theta2 = cs_data_.cos_value[n];
        double sin_theta2 = cs_data_.sin_value[n];
        endpoint1.x = -l_c*cos_theta1/(l_a*cos_theta1 + l_b*sin_theta1);
        endpoint1.y = -l_c*sin_theta1/(l_a*cos_theta1 + l_b*sin_theta1);

        // if(std::isnan(endpoint1.x) || std::isnan(endpoint1.y) ) {
        //     fprintf(stderr, "la,b,c: %f, %f, %f \n", 
        //         l_a, l_b, l_c);
        // }

        line_temp.x1 = endpoint1.x;
        line_temp.y1 = endpoint1.y;

        m_line[i].p1 = endpoint1;

        // endpoint2.x = (l_bb*range_data_.xs[n] - l_ab*range_data_.ys[n] - l_ac)/l_aa_p_bb;
        // endpoint2.y = (l_aa*range_data_.ys[n] - l_ab*range_data_.xs[n] - l_bc)/l_aa_p_bb;

        endpoint2.x = -l_c*cos_theta2/(l_a*cos_theta2 + l_b*sin_theta2);
        endpoint2.y = -l_c*sin_theta2/(l_a*cos_theta2 + l_b*sin_theta2);

        // if(std::isnan(endpoint2.x) || std::isnan(endpoint2.y) ) {
        //     fprintf(stderr, "la,b,c: %f, %f, %f \n", 
        //         l_a, l_b, l_c);
        // }

        line_temp.x2 = endpoint2.x;
        line_temp.y2 = endpoint2.y;

        m_line[i].p2 = endpoint2;

        m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

        // calculate vertical point, theta, distance.
        double vp_x, vp_y, vp_theta, vp_dist;
        vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
        vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
        vp_theta = std::atan2(vp_y, vp_x);
        vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

        m_line[i].theta = vp_theta;
        m_line[i].vdist = vp_dist;
        m_line[i].vp.x = vp_x;
        m_line[i].vp.y = vp_y;
        m_line[i].nvp.x = vp_x / vp_dist;
        m_line[i].nvp.y = vp_y / vp_dist;

        output.push_back(line_temp);
    }
    temp_line2 = output;
}

//识别主函数
void LineFeature::extractLines(std::vector<line>& temp_line1,std::vector<gline>& temp_line2)
{
    int line_include = 0;
    m_line.clear();
    point_num_ = cs_data_.index;

    if(point_num_.size() < params_.min_line_points)
    {
        return;
    }

    //附近特征点数目
    for(unsigned int i = 0; i < (m_valid_index.size() - params_.min_line_points) ;i++)
    {
        m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
        if(detectline(i,params_.seed_line_points))
        {
            int left_v_idx = m_valid_index[i];
            int right_v_idx = m_valid_index[i+params_.seed_line_points-1];
            // fprintf(stderr, "line seed detected at: (%d, %d) \n", 
            //     left_v_idx, right_v_idx);
            // fprintf(stderr, "seed left pt: (%f, %f) \n", 
            //     range_data_.xs[left_v_idx], range_data_.ys[left_v_idx]);
            // fprintf(stderr, "seed right pt: (%f, %f) \n", 
            //     range_data_.xs[right_v_idx], range_data_.ys[right_v_idx]);

            // fprintf(stderr, "line equation: %f, %f, %f \n",
            //     m_least.a, m_least.b, m_least.c);

            line_include = detectfulline(i);
            i = line_include;


        }
    }

    cleanline();

    for(int p = 0; p < m_line.size();p++)
    {
        if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
        {
            m_line.erase(m_line.begin()+p);
        }
    }
    
    generate(temp_line2);

    // delete radial lines.
    for(int p = 0; p < m_line.size();p++)
    {
        if(delete_shadow_line(m_line[p]))
        {
            m_line.erase(m_line.begin()+p);
            temp_line2.erase(temp_line2.begin()+p);
        }
    }        

    temp_line1 = m_line;
}

// void LineFeature::generate(std::vector<gline3d>& temp_line2, int laser_idx)
// {
//     gline3d line_temp;
//     std::vector<gline3d> output;
//     POINT endpoint1;
//     POINT endpoint2;
//     int m = 0,n = 0;
//     double k1 = 0,k2 = 0;
//     for(int i = 0;i < m_line.size();i++)
//     {   

//         m = m_valid_index[m_line[i].left];
//         n = m_valid_index[m_line[i].right];

//         // find middle of valid points.
//         int mid_idx = (m_line[i].left + m_line[i].right)/2;
//         int valid_mid_pt_idx = m_valid_index[mid_idx];
//         m_line[i].pm.x = range_data_.xs[valid_mid_pt_idx];
//         m_line[i].pm.y = range_data_.ys[valid_mid_pt_idx];
//         m_line[i].zm = range_data_.zs[valid_mid_pt_idx];



//         m_line[i].left = m;
//         m_line[i].right = n;
//         m_line[i].laserIdx = laser_idx;

//         calcVarianceOfFittedLine(m_line[i], m, n);

//         double line_aa = m_line[i].a*m_line[i].a;
//         double line_bb = m_line[i].b*m_line[i].b;
//         double line_ab = m_line[i].a*m_line[i].b;
//         double line_ac = m_line[i].a*m_line[i].c;
//         double line_bc = m_line[i].b*m_line[i].c;
//         double line_aa_p_bb = line_aa + line_bb;

//         endpoint1.x = (line_bb*range_data_.xs[m] - line_ab*range_data_.ys[m] - line_ac) / line_aa_p_bb;
//         endpoint1.y = (line_aa*range_data_.ys[m] - line_ab*range_data_.xs[m] - line_bc) / line_aa_p_bb;

//         line_temp.x1 = endpoint1.x;
//         line_temp.y1 = endpoint1.y;
//         line_temp.z1 = range_data_.zs[m];

//         m_line[i].p1 = endpoint1;
//         m_line[i].z1 = range_data_.zs[m];

//         endpoint1.x = (line_bb*range_data_.xs[n] - line_ab*range_data_.ys[n] - line_ac) / line_aa_p_bb;
//         endpoint1.y = (line_aa*range_data_.ys[n] - line_ab*range_data_.xs[n] - line_bc) / line_aa_p_bb; 

//         line_temp.x2 = endpoint2.x;
//         line_temp.y2 = endpoint2.y;
//         line_temp.z2 = range_data_.zs[n];

//         m_line[i].p2 = endpoint2;
//         m_line[i].z2 = range_data_.zs[n];

//         m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

//         // calculate vertical point, theta, distance.
//         double vp_x, vp_y, vp_theta, vp_dist;
//         vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
//         vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
//         vp_theta = std::atan2(vp_y, vp_x);
//         vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

//         m_line[i].theta = vp_theta;
//         m_line[i].vdist = vp_dist;
//         m_line[i].vp.x = vp_x;
//         m_line[i].vp.y = vp_y;
//         m_line[i].nvp.x = vp_x / vp_dist;
//         m_line[i].nvp.y = vp_y / vp_dist;

//         output.push_back(line_temp);
//     }
//     temp_line2 = output;
// }

// void LineFeature::extractLines(std::vector<line>& temp_line1,std::vector<gline3d>& temp_line2, int laser_idx)
// {
//     int line_include = 0;
//     m_line.clear();
//     point_num_ = cs_data_.index;

//     if(m_valid_index.size() < params_.min_line_points)
//     {
//         temp_line1.clear();
//         temp_line2.clear();
//         return;
//     }
//     //附近特征点数目

//     for(unsigned int i = 0; i < (m_valid_index.size() - params_.min_line_points) ;i++)
//     {
//         m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
//         if(detectline(i,params_.seed_line_points))
//         {
//             line_include = detectfulline(i);

//             i = line_include;
//         }

//     }

//     cleanline();

//     for(int p = 0; p < m_line.size();p++)
//     {
//         if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
//         {
//             m_line.erase(m_line.begin()+p);
//         }
//     }
    
//     generate(temp_line2, laser_idx);

//     // delete radial lines.
//     for(int p = 0; p < m_line.size();p++)
//     {
//         if(delete_shadow_line(m_line[p]))
//         {
//             m_line.erase(m_line.begin()+p);
//             temp_line2.erase(temp_line2.begin()+p);
//         }
//     }        

//     temp_line1 = m_line;
// }


// void LineFeature::extractLinesNew(std::vector<line>& temp_line1,std::vector<gline3d>& temp_line2, int laser_idx)
// {
//     int line_include = 0;
//     m_line.clear();
//     point_num_ = cs_data_.index;

//     if(m_valid_index.size() < params_.seed_line_points)
//     {
//         temp_line1.clear();
//         temp_line2.clear();
//         return;
//     }
//     //附近特征点数目

//     for(unsigned int i = 0; i < (m_valid_index.size() - params_.seed_line_points) ;i++)
//     {
//         m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
//         if(detectline(i,params_.seed_line_points))
//         {
//             line_include = detectfulline(i);

//             i = line_include;
//         }

//     }

//     cleanline();

//     for(int p = 0; p < m_line.size();p++)
//     {
//         if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
//         {
//             m_line.erase(m_line.begin()+p);
//         }
//     }
    
//     generateNew(temp_line2, laser_idx);

//     // delete radial lines.
//     for(int p = 0; p < m_line.size();p++)
//     {
//         if(delete_shadow_line(m_line[p]))
//         {
//             m_line.erase(m_line.begin()+p);
//             temp_line2.erase(temp_line2.begin()+p);
//         }
//     }        

//     temp_line1 = m_line;
// }

// void LineFeature::generateNew(std::vector<gline3d>& temp_line2, int laser_idx)
// {
//     gline3d line_temp;
//     std::vector<gline3d> output;
//     POINT endpoint1;
//     POINT endpoint2;
//     int m = 0,n = 0;
//     double k1 = 0,k2 = 0;
//     for(int i = 0;i < m_line.size();i++)
//     {   

//         m = m_valid_index[m_line[i].left];
//         n = m_valid_index[m_line[i].right];

//         // find middle of valid points.
//         int mid_idx = (m_line[i].left + m_line[i].right)/2;
//         int valid_mid_pt_idx = m_valid_index[mid_idx];
//         m_line[i].mid = valid_mid_pt_idx;

//         m_line[i].left = m;
//         m_line[i].right = n;
//         m_line[i].laserIdx = laser_idx;

//         calcVarianceOfFittedLine(m_line[i], m, n);


//         double line_aa = m_line[i].a*m_line[i].a;
//         double line_bb = m_line[i].b*m_line[i].b;
//         double line_ab = m_line[i].a*m_line[i].b;
//         double line_ac = m_line[i].a*m_line[i].c;
//         double line_bc = m_line[i].b*m_line[i].c;
//         double line_aa_p_bb = line_aa + line_bb;

//         endpoint1.x = (line_bb*range_data_.xs[m] - line_ab*range_data_.ys[m] - line_ac) / line_aa_p_bb;
//         endpoint1.y = (line_aa*range_data_.ys[m] - line_ab*range_data_.xs[m] - line_bc) / line_aa_p_bb;


//         line_temp.x1 = endpoint1.x;
//         line_temp.y1 = endpoint1.y;

//         m_line[i].p1 = endpoint1;

//         endpoint1.x = (line_bb*range_data_.xs[n] - line_ab*range_data_.ys[n] - line_ac) / line_aa_p_bb;
//         endpoint1.y = (line_aa*range_data_.ys[n] - line_ab*range_data_.xs[n] - line_bc) / line_aa_p_bb;            

//         line_temp.x2 = endpoint2.x;
//         line_temp.y2 = endpoint2.y;

//         m_line[i].p2 = endpoint2;

//         m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

//         // calculate vertical point, theta, distance.
//         double vp_x, vp_y, vp_theta, vp_dist;
//         vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
//         vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
//         vp_theta = std::atan2(vp_y, vp_x);
//         vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

//         m_line[i].theta = vp_theta;
//         m_line[i].vdist = vp_dist;
//         m_line[i].vp.x = vp_x;
//         m_line[i].vp.y = vp_y;
//         m_line[i].nvp.x = vp_x / vp_dist;
//         m_line[i].nvp.y = vp_y / vp_dist;

//         output.push_back(line_temp);
//     }
//     temp_line2 = output;
// }    


void LineFeature::calcVarianceOfFittedLine(line& l_param, int start_idx, int stop_idx) {
    double a, b, c;
    a = l_param.a; b = l_param.b; c = l_param.c;
    double sum = 0;
    double cnt = 0;
    for(int k=start_idx; k <= stop_idx; k++ ) {
        if(std::isnan(range_data_.xs[k]) || std::isnan(range_data_.ys[k]))
            continue;

        if(range_data_.ranges[k] <0.001 || range_data_.ranges[k] > 80.0)
            continue;

        double error1 = fabs((a*range_data_.xs[k]+ b*range_data_.ys[k]+c))/sqrt((a*a + b*b));
        sum += error1;
        cnt += 1.0;
    }
    sum /= cnt;
    l_param.var = sum;
}


}