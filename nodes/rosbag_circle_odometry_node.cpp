#include <ros/ros.h>

#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue
#include <rosbag/view.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tuple>

#include <pcl/common/transforms.h>


#include "circle_odometry.h"
// #include "Visualizer.h"
#include "common.h"
int last_pair_num = 0;




// set exp rate vs data rate.
float data_rate = 15;
float exp_rate = 15;
float ra = exp_rate / data_rate;
float num_pulse = 0;
float num_samples = 0;


CircleOdometry cl;

ros::Publisher pubLaserOdom;
ros::Publisher pubLaserPath;
ros::Publisher pubMarker;
ros::Publisher pubMarkerArray;
ros::Publisher pubMidPointsMarker;

ros::Publisher pubCurScanMsg;
ros::Publisher pubLastScanMsg;
ros::Publisher pubCurScaninLastFrameMsg;
ros::Publisher pubTransCloudMsg;

ros::Publisher pubHistMarker;

nav_msgs::Path laser_path;

tf::TransformBroadcaster* odom_tf_broadcaster;

bool first_frame = true;
sensor_msgs::LaserScan last_scanmsg;

std::vector<std::tuple<float, float, float>> transfrom_params;
std::vector<double> timestamps;

void laserScanMsgHandler(const sensor_msgs::LaserScan::ConstPtr& laserscanMsg) {

  num_pulse += 1;
  if (num_samples / num_pulse < ra) {
    num_samples += 1;

    cl.run(laserscanMsg);

    Eigen::Matrix3f Tsum = cl.getTransformSum();
    float heading_angle = std::atan2(Tsum(1,0), Tsum(0,0));

    transfrom_params.push_back({Tsum(0,2), Tsum(1,2), heading_angle});
    timestamps.push_back( laserscanMsg->header.stamp.toSec() );

    nav_msgs::Odometry odom;
    odom.header.stamp = laserscanMsg->header.stamp;
    odom.header.frame_id = "odom";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( heading_angle);  

    odom.pose.pose.position.x = Tsum(0,2);
    odom.pose.pose.position.y = Tsum(1,2);
    // odom.pose.pose.position.x = 0;
    // odom.pose.pose.position.y = 0;

    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "laser2";

    // upper is source, lower is target

    // cl.pubPointsMarker(pubMidPointsMarker, laserscanMsg->header.stamp);
    if(first_frame) {
      first_frame = false;
    } else {
      pcl::PointXYZ s_color, t_color;
      s_color.x = s_color.z = 0;
      s_color.y = 1.0;
      t_color.x = t_color.y = 0;
      t_color.z = 1.0;

      pubLineSegments(pubMarker, cl.getSourceGlinesVis(), 2, 1, s_color, "laser2", laserscanMsg->header.stamp);
      pubLineSegments(pubMarker, cl.getTargetGlinesVis(), 0, 2, t_color, "laser2", laserscanMsg->header.stamp);


      pubLinePairs(pubMarkerArray, 
                  cl.getMatches(), 
                  cl.getSourceGlinesVis(), 
                  cl.getTargetGlinesVis(), 
                  "laser2",
                  laserscanMsg->header.stamp);


      // pubLaserScanMsg(pubLastScanMsg, last_scanmsg, laserscanMsg->header.stamp);

      // publish aligned cloud in odom frame. 
      pcl::PointCloud<pcl::PointXYZ> trans_scan_cloud, cloud_in_odom;
      sensor_msgs::PointCloud2 cloud_msg;
      fromLaserScanToCloud2Msg(laserscanMsg, cloud_msg);
      pcl::fromROSMsg(cloud_msg, trans_scan_cloud);
      Eigen::Affine3f Tsum44 = Eigen::Affine3f::Identity();  
      Tsum44.translation() << Tsum(0,2), Tsum(1,2), 0.0;
      Tsum44.rotate (Eigen::AngleAxisf (heading_angle, Eigen::Vector3f::UnitZ()));  
      pcl::transformPointCloud(trans_scan_cloud, cloud_in_odom, Tsum44);
      publishCloudMsg(pubTransCloudMsg, cloud_in_odom, laserscanMsg->header.stamp, "odom");   

      pubHistogram(pubHistMarker, cl.getSourceHistogram(), -16,  3, s_color, "laser2", laserscanMsg->header.stamp);
      pubHistogram(pubHistMarker, cl.getTargetHistogram(),  -15, 4, t_color, "laser2", laserscanMsg->header.stamp);
      pubHistogram(pubHistMarker, cl.getTargetShiftedHistogram(), -16.5, 5, t_color, "laser2", laserscanMsg->header.stamp);

      
      pubLaserScanMsg(pubLastScanMsg, last_scanmsg, "laser2_last", laserscanMsg->header.stamp);

      sensor_msgs::LaserScan cur_scanmsg;
      cur_scanmsg = *laserscanMsg;
      pubLaserScanMsg(pubCurScanMsg, cur_scanmsg, "laser2", laserscanMsg->header.stamp);

      pubLaserScanMsg(pubCurScaninLastFrameMsg, cur_scanmsg, "laser2_tclo", laserscanMsg->header.stamp);
    }
    last_scanmsg = *laserscanMsg;

    pubLaserOdom.publish(odom);


    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = laserscanMsg->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "laser2";

    odom_trans.transform.translation.x = Tsum(0,2);
    odom_trans.transform.translation.y = Tsum(1,2);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_tf_broadcaster->sendTransform(odom_trans);

    // tf broadcast from laser2 to laser2_tclo
    geometry_msgs::TransformStamped last_to_cur_trans;

    last_to_cur_trans.header.stamp = laserscanMsg->header.stamp;
    last_to_cur_trans.header.frame_id = "laser2";
    last_to_cur_trans.child_frame_id = "laser2_tclo";

    Eigen::Matrix3f Tst = cl.getTransform();
    float rot_st = std::atan2(Tst(1,0), Tst(0,0));
    geometry_msgs::Quaternion st_quat = tf::createQuaternionMsgFromYaw(rot_st);
    last_to_cur_trans.transform.translation.x = Tst(0,2);
    last_to_cur_trans.transform.translation.y = Tst(1,2);
    last_to_cur_trans.transform.translation.z = 0.0;
    last_to_cur_trans.transform.rotation = st_quat;

    odom_tf_broadcaster->sendTransform(last_to_cur_trans);


    geometry_msgs::PoseStamped poseEst;
    poseEst.header.frame_id = "odom";
    poseEst.header.stamp =  laserscanMsg->header.stamp;
    poseEst.pose.orientation.x = 0;
    poseEst.pose.orientation.y = 0;
    poseEst.pose.orientation.z = 0;
    poseEst.pose.orientation.w = 1;
    poseEst.pose.position.x = Tsum(0,2);
    poseEst.pose.position.y = Tsum(1,2);
    poseEst.pose.position.z = 0;

    laser_path.poses.push_back(poseEst);
  }
  // pubLaserPath.publish(laser_path);



  return;

  // pcl::PointCloud<pcl::PointXY> laserCloudIn;
  // sensor_msgs::PointCloud2 cloud2msg;

  // fromLaserScanToCloud2Msg(laserscanMsg, cloud2msg);

  // pcl::fromROSMsg(cloud2msg, laserCloudIn);

  // cl.run(laserCloudIn);
}

int main(int argc, char *argv[])
{
  /* code */
  ros::init(argc, argv, "rosbag_circle_odometry_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  std::string timing_file_path;
  std::string timing_file_name;
  std::string bag_filepath;

  bool save_results_flag;

  nh_local.param<std::string>("timing_file_path", timing_file_path, "./");  
  nh_local.param<std::string>("timing_file_name", timing_file_name, "timing");  
  nh_local.param<float>("data_rate", data_rate, 15);
  nh_local.param<float>("exp_rate", exp_rate, 15);
  
  nh_local.param<std::string>("bag_filepath", bag_filepath, "");  

  nh_local.param<bool>("save_results_flag", save_results_flag, false);

  
  // update ra.
  ra = exp_rate / data_rate;

  std::string sub_laser_topic = "/scan";
  std::string pub_odom_topic = "/laser_odom";

//   ros::Subscriber subLaserScan = nh.subscribe<sensor_msgs::LaserScan>(sub_laser_topic, 2, &laserScanMsgHandler);
  rosbag::Bag bag;
  bag.open(bag_filepath);  // BagMode is Read by default

  pubLaserOdom = nh.advertise<nav_msgs::Odometry>(pub_odom_topic, 50);
  pubMarker = nh.advertise<visualization_msgs::Marker>( "planes", 1 );
  pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("plane_pairs", 1);
  pubMidPointsMarker = nh.advertise<visualization_msgs::Marker>( "mid_points", 1 );

  pubCurScanMsg  = nh.advertise<sensor_msgs::LaserScan>("/cur_scan", 20);
  pubLastScanMsg = nh.advertise<sensor_msgs::LaserScan>("/last_scan", 20);
  pubCurScaninLastFrameMsg = nh.advertise<sensor_msgs::LaserScan>("/trans_scan", 20);

  pubTransCloudMsg = nh.advertise<sensor_msgs::PointCloud2>("/trans_cloud", 20);

  pubHistMarker = nh.advertise<visualization_msgs::Marker>("histogram", 2);

  // pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_path", 5);
  laser_path.header.frame_id = "odom";


  odom_tf_broadcaster = new tf::TransformBroadcaster();

  if(argc == 1)
  {
    cl.loadParamsFromYAML("/home/bzdfzfer/CICP_ws/src/circle_odometry/config/config.yaml");
  } else {
    cl.loadParamsFromYAML(argv[1]);
  }

  // generate visualization thread.
  // boost::thread visualizer(boost::bind(&PlaneNormalVisualizer::Spin, &(cl.normal_vis)));

//   ros::Rate r(1000);

  std::vector<std::string> topics;
  topics.push_back(sub_laser_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  int frameCnt = 0;

  TicToc timer_all_scans;
  timer_all_scans.Tic();
  for(rosbag::MessageInstance const m: view)
  {
    sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
    if (i != nullptr)
    {

      frameCnt ++;
      std::cout << "Frame: " << frameCnt << std::endl;
      laserScanMsgHandler(i);
    }
  }
  ROS_INFO_STREAM("Total exection time: " << timer_all_scans.Toc() / 1000 << " seconds.");

//   while(ros::ok()) {

//     ros::spinOnce();
//     r.sleep();
//   }

  if(save_results_flag) {
      timing_file_name = timing_file_name + "_" + std::to_string((int)(exp_rate)) + "Hz";
      cl.saveTimingFile(timing_file_path, timing_file_name);

      // save trajectory
      std::string poses_file = timing_file_path + timing_file_name + "_poses.txt";
      std::ofstream f_out;
      f_out.open(poses_file);
      f_out.setf(std::ios_base::fixed);

      for(int i=0; i < transfrom_params.size(); i++) {
        f_out << std::setprecision(9) << timestamps[i] << " "
                                      << std::get<0>(transfrom_params[i]) << " " 
                                      << std::get<1>(transfrom_params[i]) << " " 
                                      << std::get<2>(transfrom_params[i]) << std::endl;
      }
      f_out.close();
      cout << "poses saved to : " << poses_file << endl;    
  }

  return 0;
}