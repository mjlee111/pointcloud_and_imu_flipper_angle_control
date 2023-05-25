#include "../include/realsense_filter/reasense_filter.h"


//pcl
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_filtering_node");
  ros::NodeHandle n;
  cout << "START" << endl;

  front_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera2/depth/color/points", 10, front_callback);
  back_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, back_callback);

  imu_sub = n.subscribe<sensor_msgs::Imu>("/imu", 10, imu_callback);

  FL_point_pub = n.advertise<sensor_msgs::PointCloud2>("/FL_point", 10);
  FR_point_pub = n.advertise<sensor_msgs::PointCloud2>("/FR_point", 10);
  BL_point_pub = n.advertise<sensor_msgs::PointCloud2>("/BL_point", 10);
  BR_point_pub = n.advertise<sensor_msgs::PointCloud2>("/BR_point", 10);

  FL_marker = n.advertise<visualization_msgs::Marker>("FL_marker", 10);
  FR_marker = n.advertise<visualization_msgs::Marker>("FR_marker", 10);
  BL_marker = n.advertise<visualization_msgs::Marker>("BL_marker", 10);
  BR_marker = n.advertise<visualization_msgs::Marker>("BR_marker", 10);

  FL_pub = n.advertise<std_msgs::Float64>("/flipper_joint_FL_position_controller/command",10);
  FR_pub = n.advertise<std_msgs::Float64>("/flipper_joint_FR_position_controller/command",10);
  BL_pub = n.advertise<std_msgs::Float64>("/flipper_joint_BL_position_controller/command",10);
  BR_pub = n.advertise<std_msgs::Float64>("/flipper_joint_BR_position_controller/command",10);

  angle_FL = n.advertise<std_msgs::Float64>("/flipper_FL", 10);
  angle_FR = n.advertise<std_msgs::Float64>("/flipper_FR", 10);
  angle_BL = n.advertise<std_msgs::Float64>("/flipper_BL", 10);
  angle_BR = n.advertise<std_msgs::Float64>("/flipper_BR", 10);

  target_angle_FL = n.advertise<std_msgs::Float64>("/target_flipper_FL", 10);
  target_angle_FR = n.advertise<std_msgs::Float64>("/target_flipper_FR", 10);
  target_angle_BL = n.advertise<std_msgs::Float64>("/target_flipper_BL", 10);
  target_angle_BR = n.advertise<std_msgs::Float64>("/target_flipper_BR", 10);

  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void front_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  three_filter(FLIPPER_FL, input_cloud_msg, FL_point_pub, -0.6, -0.05, 0.2, 0.5, -10.0, 2.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.3, -1.6808, 0, 0);
  three_filter(FLIPPER_FR, input_cloud_msg, FR_point_pub, 0.05, 0.6, 0.2, 0.5, -10.0, 2.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.3, -1.6808, 0, 0);
  marker(FLIPPER_FL, FL_marker, FL_xyz);
  marker(FLIPPER_FR, FR_marker, FR_xyz);
  float filtered_FL = MAF(atan_FL, FLIPPER_FL) - imu_roll * 0.5 + imu_pitch * 0.5;
  float filtered_FR = MAF(atan_FR, FLIPPER_FR) + imu_roll * 0.5 + imu_pitch + 0.5;
  flipper_front(filtered_FL, filtered_FR);
  //cout << "CFL :" << atan_FL << " CFR :" << atan_FR << " ";
  //cout << "FL : " << filtered_FL << " FR : " << filtered_FR << endl;
}

void back_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  three_filter(FLIPPER_BL, input_cloud_msg, BL_point_pub, 0.05 , 0.15, -0.2, 0.4, -100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.25, 2.0944, 0, 3.14159);
  three_filter(FLIPPER_BR, input_cloud_msg, BR_point_pub, -0.15, -0.05, -0.2, 0.4,-100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.25, 2.0944, 0, 3.14159);
  marker(FLIPPER_BL, BL_marker, BL_xyz);
  marker(FLIPPER_BR, BR_marker, BR_xyz);
  float filtered_BL = MAF(atan_BL, FLIPPER_BL) - imu_roll * 0.5 - imu_pitch * 0.5;
  float filtered_BR = MAF(atan_BR, FLIPPER_BR) + imu_roll * 0.5 - imu_pitch * 0.5;
  flipper_back(filtered_BL, filtered_BR);
  //cout << "CBL :" << atan_BL << " CBR :" << atan_BR << " " << endl;
  //cout << "BL : " << filtered_BL << " BR : " << filtered_BR << endl << endl;
}

void imu_callback(const sensor_msgs::Imu input_imu)
{
  Quaternion q;
  q.w = input_imu.orientation.w;
  q.x = input_imu.orientation.x;
  q.y = input_imu.orientation.y;
  q.z = input_imu.orientation.z;

  EulerAngles angles = quaternionToEulerAngles(q);

  imu_pitch = toDEG(angles.roll);
  imu_roll = toDEG(angles.pitch);
  imu_yaw = toDEG(angles.yaw);

  //cout << imu_roll << " / " << imu_pitch << endl; 
}

void three_filter(int flipper, const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg, const ros::Publisher output_pub,
                  float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                  float x_leaf, float y_leaf, float z_leaf, int meanK, float threshold,
                  float x, float y, float z, float deg_p, float deg_y, float deg_r)
{
  // passthrough X : input_cloud_msg -> filtered_cloud_x
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud_passthrough);

  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(input_cloud_passthrough);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(x_min, x_max);
  pass_filter.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter.filter(*filtered_cloud_x);

  if(filtered_cloud_x->empty()) return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // passthrough Y : filtered_cloud_x -> filtered_cloud_y

  pcl::PassThrough<pcl::PointXYZ> pass_filter2;
  pass_filter2.setInputCloud(filtered_cloud_x);
  pass_filter2.setFilterFieldName("y");
  pass_filter2.setFilterLimits(y_min, y_max);
  pass_filter2.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter2.filter(*filtered_cloud_y);

  if(filtered_cloud_y->empty()) return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // passthrough Z : filtered_cloud_y -> filtered_cloud_1

  pcl::PassThrough<pcl::PointXYZ> pass_filter3;
  pass_filter3.setInputCloud(filtered_cloud_y);
  pass_filter3.setFilterFieldName("z");
  pass_filter3.setFilterLimits(z_min, z_max);
  pass_filter3.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter3.filter(*filtered_cloud_1);

  if(filtered_cloud_1->empty()) return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // voxel : filtered_cloud_1 -> filtered_cloud_2

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(filtered_cloud_1);
  voxel_filter.setLeafSize(x_leaf, y_leaf, z_leaf);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*filtered_cloud_2);

  if(filtered_cloud_2->empty()) return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // outliner : filtered_cloud_2 -> three_filtered_cloud

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliner_filter;
  outliner_filter.setInputCloud(filtered_cloud_2);
  outliner_filter.setMeanK(meanK);
  outliner_filter.setStddevMulThresh(threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr three_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  outliner_filter.filter(*three_filtered_cloud);

  if(three_filtered_cloud->empty()) return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // tf calculation : three_filtered_cloud -> transformed_cloud

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << x, y, z;
  transform.rotate(Eigen::AngleAxisf(deg_p, Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(deg_y, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(deg_r, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*filtered_cloud_2, *transformed_cloud, transform);

  if(transformed_cloud->empty()) return;

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  sensor_msgs::PointCloud2 final;
  pcl::toROSMsg(*transformed_cloud, final);
  final.header.frame_id = "base_link";
  sensor_msgs::PointCloud2ConstPtr fiptr(new sensor_msgs::PointCloud2(final));
  output_pub.publish(final);
  max_Z(fiptr, flipper);
  return;
}

double max_Z(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, int flipper)
{
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  float max_z = -numeric_limits<float>::infinity(); // initialize to negative infinity
  float y_of_max_z = 0;
  float x_of_max_z = 0;

  for (const auto& point : cloud->points)
  {
    if (point.z > max_z)
    {
      max_z = point.z;
      y_of_max_z = point.y;
      x_of_max_z = point.x;
    }
  }
  float tan = max_z/y_of_max_z;
  float angle = atan(tan);
  angle = angle*180/M_PI;

  if(flipper == FLIPPER_FL)
  {
    atan_FL = angle;
    FL_xyz[0] = x_of_max_z;
    FL_xyz[1] = y_of_max_z;
    FL_xyz[2] = max_z;
    target_FL_msg.data = angle;
    target_angle_FL.publish(target_FL_msg);
  }
  else if(flipper == FLIPPER_FR) 
  {
    atan_FR = angle;
    FR_xyz[0] = x_of_max_z;
    FR_xyz[1] = y_of_max_z;
    FR_xyz[2] = max_z;
    target_FR_msg.data = angle;
    target_angle_FR.publish(target_FR_msg);
  }
  else if(flipper == FLIPPER_BL) 
  {
    atan_BL = -angle;
    BL_xyz[0] = x_of_max_z;
    BL_xyz[1] = y_of_max_z;
    BL_xyz[2] = max_z;
    target_BL_msg.data = angle;
    target_angle_BL.publish(target_BL_msg);
  }
  else if(flipper == FLIPPER_BR) 
  {
    atan_BR = -angle;
    BR_xyz[0] = x_of_max_z;
    BR_xyz[1] = y_of_max_z;
    BR_xyz[2] = max_z;
    target_BR_msg.data = angle;
    target_angle_BR.publish(target_BR_msg);
  }
  return max_z;
}

void marker(int flipper, const ros::Publisher pub,  float input_float[3])
{
  visualization_msgs::Marker line;
  line.header.frame_id = "base_link";
  line.header.stamp = ros::Time::now();
  line.ns = "line";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP; // Set the type of the marker to a line strip (i.e. a series of connected line segments)
  line.action = visualization_msgs::Marker::ADD; // Set the action of the marker to add it to the display
  line.scale.x = 0.01; // Set the width of the line segments

  // Set the start and end points of the line
  geometry_msgs::Point p1, p2;

  if(flipper == FLIPPER_FL || flipper == FLIPPER_BL) p1.x = -0.75-0.15;
  else if(flipper == FLIPPER_FR || flipper == FLIPPER_BR) p1.x = 0.75-0.15;
  p1.y = 0;
  p1.z = 0;
  p2.x = input_float[0];
  p2.y = input_float[1];
  p2.z = input_float[2];
  line.points.push_back(p2);
  line.points.push_back(p1);

  // Set the color of the marker to green
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;
  line.color.a = 1.0;
  pub.publish(line);
}

void flipper_back(float angle_L, float angle_R)
{
  if(angle_L > MAX_F_FLIPPER) angle_L = MAX_F_FLIPPER;
  if(angle_L < MIN_F_FLIPPER) angle_L = MIN_F_FLIPPER;
  if(angle_R > MAX_F_FLIPPER) angle_R = MAX_F_FLIPPER;
  if(angle_R < MIN_F_FLIPPER) angle_R = MIN_F_FLIPPER;

  // if(angle_L > 60 || (angle_L > -10 && angle_L < 3)) angle_L = 90;
  // if(angle_R > 60 || (angle_R > -10 && angle_R < 3)) angle_R = 90;

  target_BL= -angle_L * M_PI / 180;
  target_BR= angle_R * M_PI / 180;

  if(now_BL > target_BL + FLIPPER_SPEED_GAIN) now_BL -= FLIPPER_SPEED_GAIN;
  else if(now_BL < target_BL - FLIPPER_SPEED_GAIN) now_FL += FLIPPER_SPEED_GAIN;
  else if((now_BL > target_BL -FLIPPER_SPEED_GAIN)&&(now_BL<target_BL+FLIPPER_SPEED_GAIN)) now_BL=target_BL;

  if(now_BR > target_BR + FLIPPER_SPEED_GAIN) now_BR -= FLIPPER_SPEED_GAIN;
  else if(now_BR < target_BR - FLIPPER_SPEED_GAIN) now_BR += FLIPPER_SPEED_GAIN;
  else if((now_BR > target_BR -FLIPPER_SPEED_GAIN)&&(now_BR<target_BR+FLIPPER_SPEED_GAIN)) now_BR=target_BR;

  BL.data = now_BL;
  BR.data = now_BR;
  BL_pub.publish(BR);
  BR_pub.publish(BL);
  angle_BL.publish(BR);
  angle_BR.publish(BL);
}

void flipper_front(float angle_L, float angle_R)
{
  if(angle_L > MAX_F_FLIPPER) angle_L = MAX_F_FLIPPER;
  if(angle_L < MIN_F_FLIPPER) angle_L = MIN_F_FLIPPER;
  if(angle_R > MAX_F_FLIPPER) angle_R = MAX_F_FLIPPER;
  if(angle_R < MIN_F_FLIPPER) angle_R = MIN_F_FLIPPER;

  // if(angle_L > 60 || (angle_L > -10 && angle_L < 5)) angle_L = 120;
  // if(angle_R > 60 || (angle_R > -10 && angle_R < 5)) angle_R = 120;

  target_FL= angle_L * M_PI / 180;
  target_FR= -angle_R * M_PI / 180;

  if(now_FL > target_FL + FLIPPER_SPEED_GAIN) now_FL -= FLIPPER_SPEED_GAIN;
  else if(now_FL < target_FL - FLIPPER_SPEED_GAIN) now_FL += FLIPPER_SPEED_GAIN;
  else if((now_FL > target_FL -FLIPPER_SPEED_GAIN)&&(now_FL<target_FL+FLIPPER_SPEED_GAIN)) now_FL=target_FL;

  if(now_FR > target_FR + FLIPPER_SPEED_GAIN) now_FR -= FLIPPER_SPEED_GAIN;
  else if(now_FR < target_FR - FLIPPER_SPEED_GAIN) now_FR += FLIPPER_SPEED_GAIN;
  else if((now_FR > target_FR -FLIPPER_SPEED_GAIN)&&(now_FR<target_FR+FLIPPER_SPEED_GAIN)) now_FR=target_FR;

  FL.data = now_FL;
  FR.data = now_FR;
  FL_pub.publish(FR);
  FR_pub.publish(FL);
  angle_FL.publish(FR);
  angle_FR.publish(FL);
}

float MAF(float input, int flipper)
{
  if(flipper == FLIPPER_FL)
  {
    maf_input_array_FL[maf_index_FL] = input;
    maf_index_FL ++;
    if(maf_index_FL >= MAF_MASK_SIZE)maf_index_FL = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += maf_input_array_FL[i];
    return sum/MAF_MASK_SIZE;
  }

  else if(flipper == FLIPPER_FR)
  {
    maf_input_array_FR[maf_index_FR] = input;
    maf_index_FR ++;
    if(maf_index_FR >= MAF_MASK_SIZE)maf_index_FR = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += maf_input_array_FR[i];
    return sum/MAF_MASK_SIZE;
  }

  else if(flipper == FLIPPER_BL)
  {
    maf_input_array_BL[maf_index_BL] = input;
    maf_index_BL ++;
    if(maf_index_BL >= MAF_MASK_SIZE)maf_index_BL = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += maf_input_array_BL[i];
    return sum/MAF_MASK_SIZE;
  }

  else if(flipper == FLIPPER_BR)
  {
    maf_input_array_BR[maf_index_BR] = input;
    maf_index_BR ++;
    if(maf_index_BR >= MAF_MASK_SIZE)maf_index_BR = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += maf_input_array_BR[i];
    return sum/MAF_MASK_SIZE;
  }
  else if(flipper == ROLL)
  {
    imu_roll_MAF[maf_index_roll] = input;
    maf_index_roll ++;
    if(maf_index_roll >= MAF_MASK_SIZE)maf_index_roll = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += imu_roll_MAF[i];
    return sum/MAF_MASK_SIZE;
  }
  else
  {
    imu_pitch_MAF[maf_index_pitch] = input;
    maf_index_pitch ++;
    if(maf_index_pitch >= MAF_MASK_SIZE)maf_index_pitch = 0;
    float sum = 0;
    for(int i = 0 ; i < MAF_MASK_SIZE ; i++) sum += imu_pitch_MAF[i];
    return sum/MAF_MASK_SIZE;
  }
}

float toRAD(float deg)
{
  return deg * M_PI / 180;
}

float toDEG(float rad)
{
  return rad * 180 / M_PI;
}

double ransac(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  // Convert the input point cloud message to a PointCloudXYZ object
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input_cloud_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Define the model to represent the YZ plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  coefficients->values.resize(3);
  coefficients->values[0] = 0; // a = 0
  coefficients->values[1] = 1; // b = 1
  coefficients->values[2] = 0; // c = 0

  // Create the SAC segmentation object and set the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.01);

  // Segment the largest planar component from the cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Calculate the YZ inclination
  double inclination = atan2(coefficients->values[0], coefficients->values[2]) * 180.0 / M_PI;
  return inclination;
}

EulerAngles quaternionToEulerAngles(const Quaternion& q)
{
  EulerAngles angles;

    float sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0)
        angles.pitch = std::copysign(M_PI / 2.0, sinp); 
    else
        angles.pitch = std::asin(sinp);

    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
