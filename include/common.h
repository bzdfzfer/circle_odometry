//
// Created by bzdfzfer on 2022/8/26.
//

#ifndef COMMON_H
#define COMMON_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <limits>

#include "histogram.h"
#include "fstruct.h"


#define FLOAT_NAN std::numeric_limits<float>::quiet_NaN()
#define RAD2DEG_RATIO 180.0/M_PI
#define DEG2RAD_RATIO M_PI/180.0

constexpr std::array<std::array<int, 3>, 200> RANDOM_COLORS = {{
  {{104, 109, 253}}, {{125, 232, 153}}, {{158, 221, 134}},
  {{228, 109, 215}}, {{249, 135, 210}}, {{255, 207, 237}},
  {{151, 120, 235}}, {{145, 123, 213}}, {{172, 243, 184}},
  {{105, 131, 110}}, {{217, 253, 154}}, {{250, 102, 109}},
  {{116, 179, 127}}, {{200, 251, 206}}, {{117, 146, 240}},
  {{234, 162, 176}}, {{160, 172, 171}}, {{205, 129, 168}},
  {{197, 167, 238}}, {{234, 248, 101}}, {{226, 240, 119}},
  {{189, 211, 231}}, {{226, 170, 216}}, {{109, 180, 162}},
  {{115, 167, 221}}, {{162, 134, 131}}, {{203, 169, 114}},
  {{221, 138, 114}}, {{246, 146, 237}}, {{200, 167, 244}},
  {{198, 150, 236}}, {{237, 235, 191}}, {{132, 137, 171}},
  {{136, 219, 103}}, {{229, 210, 135}}, {{133, 188, 111}},
  {{142, 144, 142}}, {{122, 189, 120}}, {{127, 142, 229}},
  {{249, 147, 235}}, {{255, 195, 148}}, {{202, 126, 227}},
  {{135, 195, 159}}, {{139, 173, 142}}, {{123, 118, 246}},
  {{254, 186, 204}}, {{184, 138, 221}}, {{112, 160, 229}},
  {{243, 165, 249}}, {{200, 194, 254}}, {{172, 205, 151}},
  {{196, 132, 119}}, {{240, 251, 116}}, {{186, 189, 147}},
  {{154, 162, 144}}, {{178, 103, 147}}, {{139, 188, 175}},
  {{156, 163, 178}}, {{225, 244, 174}}, {{118, 227, 101}},
  {{176, 178, 120}}, {{113, 105, 164}}, {{137, 105, 123}},
  {{144, 114, 196}}, {{163, 115, 216}}, {{143, 128, 133}},
  {{221, 225, 169}}, {{165, 152, 214}}, {{133, 163, 101}},
  {{212, 202, 171}}, {{134, 255, 128}}, {{217, 201, 143}},
  {{213, 175, 151}}, {{149, 234, 191}}, {{242, 127, 242}},
  {{152, 189, 230}}, {{152, 121, 249}}, {{234, 253, 138}},
  {{152, 234, 147}}, {{171, 195, 244}}, {{254, 178, 194}},
  {{205, 105, 153}}, {{226, 234, 202}}, {{153, 136, 236}},
  {{248, 242, 137}}, {{162, 251, 207}}, {{152, 126, 144}},
  {{180, 213, 122}}, {{230, 185, 113}}, {{118, 148, 223}},
  {{162, 124, 183}}, {{180, 247, 119}}, {{120, 223, 121}},
  {{252, 124, 181}}, {{254, 174, 165}}, {{188, 186, 210}},
  {{254, 137, 161}}, {{216, 222, 120}}, {{215, 247, 128}},
  {{121, 240, 179}}, {{135, 122, 215}}, {{255, 131, 237}},
  {{224, 112, 171}}, {{167, 223, 219}}, {{103, 200, 161}},
  {{112, 154, 156}}, {{170, 127, 228}}, {{133, 145, 244}},
  {{244, 100, 101}}, {{254, 199, 148}}, {{120, 165, 205}},
  {{112, 121, 141}}, {{175, 135, 134}}, {{221, 250, 137}},
  {{247, 245, 231}}, {{236, 109, 115}}, {{169, 198, 194}},
  {{196, 195, 136}}, {{138, 255, 145}}, {{239, 141, 147}},
  {{194, 220, 253}}, {{149, 209, 204}}, {{241, 127, 132}},
  {{226, 184, 108}}, {{222, 108, 147}}, {{109, 166, 185}},
  {{152, 107, 167}}, {{153, 117, 222}}, {{165, 171, 214}},
  {{189, 196, 243}}, {{248, 235, 129}}, {{120, 198, 202}},
  {{223, 206, 134}}, {{175, 114, 214}}, {{115, 196, 189}},
  {{157, 141, 112}}, {{111, 161, 201}}, {{207, 183, 214}},
  {{201, 164, 235}}, {{168, 187, 154}}, {{114, 176, 229}},
  {{151, 163, 221}}, {{134, 160, 173}}, {{103, 112, 168}},
  {{209, 169, 218}}, {{137, 220, 119}}, {{168, 220, 210}},
  {{182, 192, 194}}, {{233, 187, 120}}, {{223, 185, 160}},
  {{120, 232, 147}}, {{165, 169, 124}}, {{251, 159, 129}},
  {{182, 114, 178}}, {{159, 116, 158}}, {{217, 121, 122}},
  {{106, 229, 235}}, {{164, 208, 214}}, {{180, 178, 142}},
  {{110, 206, 136}}, {{238, 152, 205}}, {{109, 245, 253}},
  {{213, 232, 131}}, {{215, 134, 100}}, {{163, 140, 135}},
  {{233, 198, 143}}, {{221, 129, 224}}, {{150, 179, 137}},
  {{171, 128, 119}}, {{210, 245, 246}}, {{209, 111, 161}},
  {{237, 133, 194}}, {{166, 157, 255}}, {{191, 206, 225}},
  {{125, 135, 110}}, {{199, 188, 196}}, {{196, 101, 202}},
  {{237, 211, 167}}, {{134, 118, 177}}, {{110, 179, 126}},
  {{196, 182, 196}}, {{150, 211, 218}}, {{162, 118, 228}},
  {{150, 209, 185}}, {{219, 151, 148}}, {{201, 168, 104}},
  {{237, 146, 123}}, {{234, 163, 146}}, {{213, 251, 127}},
  {{227, 152, 214}}, {{230, 195, 100}}, {{136, 117, 222}},
  {{180, 132, 173}}, {{112, 226, 113}}, {{198, 155, 126}},
  {{149, 255, 152}}, {{223, 124, 170}}, {{104, 146, 255}},
  {{113, 205, 183}}, {{100, 156, 216}},
}};


inline int wrap_idx(int idx, int MAX_SIZE) {
  int out_idx = idx;
  if(idx < 0)
    out_idx += MAX_SIZE;
  else if(idx>= MAX_SIZE)
    out_idx -= MAX_SIZE;
  return out_idx;
}


template <typename T>
inline std::vector<int> voting1DVec(const std::vector<T>& vec_1d, const T& noise_bound) {
  int N = vec_1d.size();
  
  std::vector<T> data_interval_edges;
  for(int i=0; i < N; i++) {
    T m_minus_n = vec_1d[i] - noise_bound;
    T m_plus_n = vec_1d[i] + noise_bound;
    data_interval_edges.push_back(m_minus_n);
    data_interval_edges.push_back(m_plus_n);
  }
  std::sort(data_interval_edges.begin(), data_interval_edges.end());

  // cout << "mid of intervals: " << endl;
  // get middle value of interval.
  std::vector<T> data_interval_mids(2*N-1);
  for(int i=0; i < 2*N-1; i++) {
    T mid_val = (data_interval_edges[i] + data_interval_edges[i+1])/2;
    data_interval_mids[i] = mid_val;
    // cout << mid_val << ", ";
  }
  // cout << endl;

  // voting mid values.
  std::vector<std::vector<int>> voting_index_sets(2*N-1);
  std::vector<int> voting_counts(2*N-1, 0);
  T max_consensus_cnt = 0;
  int max_consensus_idx = -1;
  // std::vector<bool> voted_flag(N, false);
  for(int i=0; i < data_interval_mids.size(); i++) {
    T mid_val = data_interval_mids[i];
    for(int j=0; j < N; j++) {  
    // if(voted_flag[j])
    //  continue; 
      T m_minus_n = vec_1d[j] - noise_bound;
      T m_plus_n = vec_1d[j] + noise_bound;
      if(mid_val >= m_minus_n && mid_val < m_plus_n) {
        // voted_flag[j] = true;
        // cout << "i,j:" << i << ", " << j  << endl;
        // cout << vec_1d[j] <<endl;
        // cout << "mid_val: " << mid_val << ", "
        //  << ", m_minus_n: "  << m_minus_n 
        //  << "m_plus_n: " << m_plus_n  << endl;
        voting_index_sets[i].push_back(j);
        voting_counts[i] ++;
        if(max_consensus_cnt < voting_counts[i]) {
          max_consensus_cnt = voting_counts[i];
          max_consensus_idx = i;
        }
      }
    }
  }

  // cout << "max_consensus_cnt: " << max_consensus_cnt << endl;
  // cout << "max_consensus_idx: " << max_consensus_idx << endl;

  std::vector<int> consensus_set;
  if(max_consensus_idx>=0) {
    consensus_set = voting_index_sets[max_consensus_idx];
  }


  return consensus_set;
}


inline void fromLaserScanToCloud2Msg(const sensor_msgs::LaserScan::ConstPtr &msg, sensor_msgs::PointCloud2& pcl2_laser)
{
    sensor_msgs::PointCloud pcl_laser;
    
    std::vector<geometry_msgs::Point32> pts;
    geometry_msgs::Point32 pt;
    float anglestart = msg->angle_min;
    for(size_t i=0; i<msg->ranges.size(); i++)
    {
        if(msg->ranges[i]>=msg->range_min && msg->ranges[i]<=msg->range_max)
        {
            pt.x = msg->ranges[i]*cos(anglestart+i*msg->angle_increment);
            pt.y = msg->ranges[i]*sin(anglestart+i*msg->angle_increment);
            pt.z = 0;
            pts.push_back(pt);
        }
        else
        {
            pt.x = FLOAT_NAN;
            pt.y = FLOAT_NAN;
            pt.z = FLOAT_NAN;
            pts.push_back(pt);
        }
    }

    pcl_laser.header = msg->header;
    pcl_laser.points = pts;

    sensor_msgs::convertPointCloudToPointCloud2(pcl_laser,pcl2_laser);
}

/** \brief Construct a new point cloud message from the specified information and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param publisher the publisher instance
 * @param cloud the cloud to publish
 * @param stamp the time stamp of the cloud message
 * @param frameID the message frame ID
 */
template <typename PointT>
inline void publishCloudMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frameID) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frameID;
  publisher.publish(msg);
}

template <typename PointT>
inline void pubCloudScanMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frameID,
                            double angle_min,
                            double angle_max,
                            double angle_increment,
                            double range_min,
                            double range_max) {
  sensor_msgs::LaserScan scan_msg;

  scan_msg.header.stamp = stamp;
  scan_msg.header.frame_id = frameID;

  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = angle_increment;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.07;
  scan_msg.range_min = range_min;
  scan_msg.range_max = range_max;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
  
  scan_msg.ranges.assign(ranges_size, std::numeric_limits<double>::quiet_NaN());

  for(int i=0; i < cloud.size(); i++) {
    if(std::isnan(cloud[i].x) || std::isnan(cloud[i].y)) {
        std::numeric_limits<double>::quiet_NaN();
    } else {
        double angle = std::atan2(cloud[i].y, cloud[i].x);
        int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;        
        scan_msg.ranges[index] = std::sqrt(cloud[i].x*cloud[i].x + cloud[i].y*cloud[i].y);
    }
  }

  publisher.publish(scan_msg);
}



inline void pubLaserScanMsg(const ros::Publisher& scan_pub, 
  sensor_msgs::LaserScan& laser_scanmsg, 
  std::string frameID,
  ros::Time timestamp) {

  // change last message 
  laser_scanmsg.header.frame_id = frameID;
  laser_scanmsg.header.stamp = timestamp;

  scan_pub.publish(laser_scanmsg);
}


//----------------------Visualize segments---------------------------------------------

inline bool checkNanPoint(const geometry_msgs::Point& geo_pt) {
  if(std::isnan(geo_pt.x) ||
     std::isnan(geo_pt.y) ||
     std::isnan(geo_pt.z) ) {

    fprintf(stderr, "%.3f , %.3f, %.3f \n", geo_pt.x, 
      geo_pt.y, geo_pt.z );
  
    return true;
  }
  else {
    return false;
  }
}

inline void pubLineSegments(const ros::Publisher& marker_pub, 
                  const std::vector<gline>& glines, 
                  float height_z,
                  int linelist_id,
                  pcl::PointXYZ color,
                  std::string frameID,
                  ros::Time timestamp) {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frameID;
    line_list.header.stamp = timestamp;
    line_list.ns = "lines";
    line_list.pose.position.x = 0.0;
    line_list.pose.position.y = 0.0;
    line_list.pose.position.z = 0.0;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.id = linelist_id;
    line_list.color.a = 1.0;
    line_list.color.r = color.x;
    line_list.color.g = color.y;
    line_list.color.b = color.z;

    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    geometry_msgs::Point point1, point2;
    line_list.points.clear();
    for (auto & line : glines){
        point1.x = line.x1;
        point1.y = line.y1;
        point1.z = point2.z = height_z;
        point2.x = line.x2;
        point2.y = line.y2;
        if(checkNanPoint(point1))
          continue;
        if(checkNanPoint(point2))
          continue;
        // cout << "line pt1.x, pt1.y, pt2.x, pt2.y:   " << point1.x << ", "
        //  << point1.y << ", " << point2.x << ", " << point2.y << endl;
        line_list.points.push_back(point1);
        line_list.points.push_back(point2);
    }
    marker_pub.publish(line_list);
}
//--------------------------------------------------------------------------------------



//----------------------Visualize segment pairs---------------------------------------------
extern int last_pair_num;
inline void pubLinePairs(const ros::Publisher& marker_pub, 
                  const std::vector<std::pair<int, int>>& matches,
                  const std::vector<gline>& s_g_lines,
                  const std::vector<gline>& t_g_lines,
                  std::string frameID,
                  ros::Time timestamp) {

  visualization_msgs::MarkerArray ma;
  int cnt = -1;
  int pair_num = matches.size();
  for (auto & p_pair : matches){
    cnt ++;
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frameID;
    line_list.header.stamp = timestamp;
    line_list.ns = "line_pairs";
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2*cnt+1;
    line_list.color.a = 1.0;
    auto random_color = RANDOM_COLORS[cnt%200];
    line_list.color.r = 1.0*random_color[0]/256;
    line_list.color.g = 1.0*random_color[1]/256;
    line_list.color.b = 1.0*random_color[2]/256;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.scale.y = 1;
    line_list.scale.z = 1;   
    geometry_msgs::Point point1, point2;

    // adding source line.
    point1.x = s_g_lines[p_pair.first].x1;
    point1.y = s_g_lines[p_pair.first].y1;
    point1.z = point2.z = 2;
    point2.x = s_g_lines[p_pair.first].x2;
    point2.y = s_g_lines[p_pair.first].y2;


    line_list.points.push_back(point1);
    line_list.points.push_back(point2);

    // adding target line.
    point1.x = t_g_lines[p_pair.second].x1;
    point1.y = t_g_lines[p_pair.second].y1;
    point2.x = t_g_lines[p_pair.second].x2;
    point2.y = t_g_lines[p_pair.second].y2;
    point1.z = point2.z = 0;

    line_list.points.push_back(point1);
    line_list.points.push_back(point2);    

      // ma.markers.push_back(line_list);            

    visualization_msgs::Marker c_line_list;
    c_line_list.header.frame_id = frameID;
    c_line_list.header.stamp = timestamp;
    c_line_list.ns = "line_pairs";
    c_line_list.pose.orientation.w = 1.0;
    c_line_list.id = 2*cnt;
    c_line_list.color.a = 1.0;
    c_line_list.color.r = 1.0;
    c_line_list.color.g = 0.0;
    c_line_list.color.b = 0.0;
    c_line_list.type = visualization_msgs::Marker::LINE_LIST;
      c_line_list.action = visualization_msgs::Marker::ADD;
    c_line_list.scale.x = 0.15;
    c_line_list.scale.y = 0.15;
    c_line_list.scale.z = 0.15;

    // adding start matching line connecting source and target.
    point1.x = s_g_lines[p_pair.first].x1;
    point1.y = s_g_lines[p_pair.first].y1;
    point2.x = t_g_lines[p_pair.second].x1;
    point2.y = t_g_lines[p_pair.second].y1;
    point1.z  = 2;
    point2.z = 0;

    c_line_list.points.push_back(point1);
    c_line_list.points.push_back(point2);  

    // adding stop matching line connecting source and target.
    point1.x = s_g_lines[p_pair.first].x2;
    point1.y = s_g_lines[p_pair.first].y2;
    point2.x = t_g_lines[p_pair.second].x2;
    point2.y = t_g_lines[p_pair.second].y2;
    point1.z = 2;
    point2.z = 0;

    c_line_list.points.push_back(point1);
    c_line_list.points.push_back(point2);  

    ma.markers.push_back(c_line_list);            
  }

  cnt ++;
  for(; cnt < last_pair_num; cnt ++) {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frameID;
    line_list.header.stamp = timestamp;
    line_list.ns = "line_pairs";
    line_list.id = 2*cnt+1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::DELETE;

    // ma.markers.push_back(line_list);            

    visualization_msgs::Marker c_line_list;
    c_line_list.header.frame_id = frameID;
    c_line_list.header.stamp = timestamp;
    c_line_list.ns = "line_pairs";
    c_line_list.id = 2*cnt;
    c_line_list.type = visualization_msgs::Marker::LINE_LIST;
    c_line_list.action = visualization_msgs::Marker::DELETE;

    ma.markers.push_back(c_line_list);            
  }

  marker_pub.publish(ma);
  
  last_pair_num = pair_num;
}
//--------------------------------------------------------------------------------------



inline void pubPointsMarker(const ros::Publisher& maker_pub, 
                    std::vector<std::pair<double, double>> points,
                    std::string frameID,
                    ros::Time timestamp) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameID;
  marker.header.stamp = timestamp;
  marker.ns = "points";
  marker.pose.orientation.w = 1.0;
  marker.id = 1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.type = visualization_msgs::Marker::POINTS; 
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  for(int i=0; i < points.size(); i++) {
    float mid_px = points[i].first;
    float mid_py = points[i].second;

    if(std::isnan(mid_px) || std::isnan(mid_py))
      continue;

    geometry_msgs::Point pt;
    pt.x = mid_px;
    pt.y = mid_py;
    pt.z = 0;
    marker.points.push_back(pt);
  }

  maker_pub.publish(marker);
}



inline void pubHistogram(const ros::Publisher& marker_pub,
                  const Histogram& hist,
                  float disp_y,
                  int linelist_id,
                  pcl::PointXYZ color,
                  std::string frameID,
                  ros::Time timestamp) 
{
  //----------------------Visualize histogram ----------------------------------
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frameID;
  line_list.header.stamp = timestamp;
  line_list.ns = "stem_lines";
  line_list.pose.position.x = 0.0;
  line_list.pose.position.y = 0.0;
  line_list.pose.position.z = 0.0;
  line_list.pose.orientation.w = 1.0;
  line_list.pose.orientation.x = 0.0;
  line_list.pose.orientation.y = 0.0;
  line_list.pose.orientation.z = 0.0;
  line_list.id = linelist_id;
  // line_list.color.a = 1.0;
  line_list.color.a = 0.75;
  line_list.color.r = color.x;
  line_list.color.g = color.y;
  line_list.color.b = color.z;

  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.03;
  line_list.scale.y = 0.03;
  line_list.scale.z = 0.03;

  geometry_msgs::Point point1, point2;
  line_list.points.clear(); 

  // loop
  std::vector<int> valid_bins = hist.getValidBins();
  for(int i=0; i < valid_bins.size(); i++) {
    double bin_idx = valid_bins[i];
    double bin_value = hist.getBinValueAt(bin_idx);
    point1.x = bin_idx/30.0;
    point1.y = disp_y;
    point1.z = point2.z = 0;
    point2.x = bin_idx/30.0;
    point2.y = bin_value/4.0+disp_y;
      
    line_list.points.push_back(point1);
    line_list.points.push_back(point2);
  }

  marker_pub.publish(line_list);
}



inline void loadLaserCloudFromTxt(std::string file_name, pcl::PointCloud<pcl::PointXY>& laser_cloud) {

  laser_cloud.clear();

  std::ifstream f_in;
  f_in.open(file_name.c_str());

  std::string line;
  while(std::getline(f_in, line)) {
    std::stringstream ss(line);
    pcl::PointXY point;
    ss >> point.x;
    ss >> point.y;
    laser_cloud.push_back(point);
  }
  f_in.close();
}



#endif