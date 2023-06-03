#include "../include/realsense_filter/reasense_filter.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_filtering_node");
  ros::NodeHandle n;
  cout << "START" << endl;

  std::string param;
  n.getParam("/realsense_filter_node/realsese_visuals", param);
  if(param == "off") 
  {
    marker_arg = false;
  }
  else if(param == "on")
  {
    marker_arg = true;
  }

  thread pointcloud_run(calthreadFunction, argc, argv);

  imu_sub = n.subscribe<sensor_msgs::Imu>("/imu", 10, imu_callback);

  // PUBLISHER
  // FLIPPER_ANGLE_DATA_PUB (RAD)
  Front_angle = n.advertise<std_msgs::Float64MultiArray>("/flipper_front", 10);
  Back_angle = n.advertise<std_msgs::Float64MultiArray>("/flipper_back", 10);

  if (marker_arg)
  {
    // FILTERED_POINTCLOUD_PUB
    FL_point_pub = n.advertise<sensor_msgs::PointCloud2>("/FL_point", 10);
    FR_point_pub = n.advertise<sensor_msgs::PointCloud2>("/FR_point", 10);
    BL_point_pub = n.advertise<sensor_msgs::PointCloud2>("/BL_point", 10);
    BR_point_pub = n.advertise<sensor_msgs::PointCloud2>("/BR_point", 10);

    // VISUALIZATION_TEXT_PUB
    FL_marker_text = n.advertise<visualization_msgs::Marker>("/FL_text", 10);
    FR_marker_text = n.advertise<visualization_msgs::Marker>("/FR_text", 10);
    BL_marker_text = n.advertise<visualization_msgs::Marker>("/BL_text", 10);
    BR_marker_text = n.advertise<visualization_msgs::Marker>("/BR_text", 10);

    // VISUALIZATION_MARKER_PUB
    FL_marker = n.advertise<visualization_msgs::Marker>("FL_marker", 10);
    FR_marker = n.advertise<visualization_msgs::Marker>("FR_marker", 10);
    BL_marker = n.advertise<visualization_msgs::Marker>("BL_marker", 10);
    BR_marker = n.advertise<visualization_msgs::Marker>("BR_marker", 10);
  }

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  pointcloud_run.join();

  return 0;
}

void calthreadFunction(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud math");
  ros::NodeHandle node;
  // SUBSCRIBER
  front_cloud_sub = node.subscribe<sensor_msgs::PointCloud2>("/camera2/depth/color/points", 10, front_callback);
  back_cloud_sub = node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, back_callback);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void front_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  // FRONT_FLIPPER_CALLBACK
  // THREE_FILTER
  three_filter(FLIPPER_FL, input_cloud_msg, FL_point_pub, -0.15, -0.05, -0.2, 0.8, -100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0.1, 0, 0.57, -2.18166, 0, 0);
  three_filter(FLIPPER_FR, input_cloud_msg, FR_point_pub, 0.05, 0.15, -0.2, 0.8, -100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0.1, 0, 0.57, -2.18166, 0, 0);
  // RVIZ
  if (marker_arg == true)
  {
    marker(FLIPPER_FL, FL_marker, FL_marker_text, flipper_xyz[FLIPPER_FL]);
    marker(FLIPPER_FR, FR_marker, FR_marker_text, flipper_xyz[FLIPPER_FR]);
  }

  // MAF
  float filtered_FL = MAF(atan_data, FLIPPER_FL) + (IMU_DATA_RELIANCE * (-(imu_roll * 0.5) + (imu_pitch * 0.5)));
  float filtered_FR = MAF(atan_data, FLIPPER_FR) + (IMU_DATA_RELIANCE * ((imu_roll * 0.5) + (imu_pitch * 0.5)));

  auto_flipper_trigger(FLIPPER_FL, FLIPPER_FR);

  flipper_front(filtered_FL, filtered_FR);
}

void back_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  three_filter(FLIPPER_BL, input_cloud_msg, BL_point_pub, 0.05, 0.15, -0.2, 0.8, -100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.255, 2.0944, 0, 3.14159);
  three_filter(FLIPPER_BR, input_cloud_msg, BR_point_pub, -0.15, -0.05, -0.2, 0.8, -100.0, 100.0, 0.06, 0.06, 0.06, 50, 1.0, 0, 0, 0.255, 2.0944, 0, 3.14159);
  if (marker_arg == true)
  {
    marker(FLIPPER_BL, BL_marker, BL_marker_text, flipper_xyz[FLIPPER_BL]);
    marker(FLIPPER_BR, BR_marker, BR_marker_text, flipper_xyz[FLIPPER_BR]);
  }

  float filtered_BL = MAF(atan_data, FLIPPER_BL) + (IMU_DATA_RELIANCE * (-(imu_roll * 0.5) - (imu_pitch * 0.5)));
  float filtered_BR = MAF(atan_data, FLIPPER_BR) + (IMU_DATA_RELIANCE * ((imu_roll * 0.5) - (imu_pitch * 0.5)));

  auto_flipper_trigger(FLIPPER_BL, FLIPPER_BR);

  flipper_back(filtered_BL, filtered_BR);
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
}

void auto_flipper_trigger(int flipper1, int flipper2)
{
  if (flipper_xyz[flipper1][2] > 0.1 && flipper_xyz[flipper1][2] < 0.5)
    max_z_cnt[flipper1] += 1;
  else
    max_z_cnt[flipper1] = 0;
  if (max_z_cnt[flipper1] > AUTO_FLIPPER_TRIGGER)
    max_z_cnt[flipper1] = AUTO_FLIPPER_TRIGGER;

  if (flipper_xyz[flipper2][2] > 0.1 && flipper_xyz[flipper2][2] < 0.5)
    max_z_cnt[flipper2] += 1;
  else
    max_z_cnt[flipper2] = 0;
  if (max_z_cnt[flipper2] > AUTO_FLIPPER_TRIGGER)
    max_z_cnt[flipper2] = AUTO_FLIPPER_TRIGGER;

  if (imu_roll < 3 && imu_roll > -3 && imu_pitch < 3 && imu_pitch > -3)
  {
    if (max_z_cnt[flipper1] >= AUTO_FLIPPER_TRIGGER)
      auto_trigger[flipper1] = true;
    else
      auto_trigger[flipper1] = false;

    if (max_z_cnt[flipper2] >= AUTO_FLIPPER_TRIGGER)
      auto_trigger[flipper2] = true;
    else
      auto_trigger[flipper2] = false;
  }
  else
    auto_trigger[flipper1, flipper2] = true;
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

  if (filtered_cloud_x->empty())
    return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // passthrough Y : filtered_cloud_x -> filtered_cloud_y

  pcl::PassThrough<pcl::PointXYZ> pass_filter2;
  pass_filter2.setInputCloud(filtered_cloud_x);
  pass_filter2.setFilterFieldName("y");
  pass_filter2.setFilterLimits(y_min, y_max);
  pass_filter2.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter2.filter(*filtered_cloud_y);

  if (filtered_cloud_y->empty())
    return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // passthrough Z : filtered_cloud_y -> filtered_cloud_1

  pcl::PassThrough<pcl::PointXYZ> pass_filter3;
  pass_filter3.setInputCloud(filtered_cloud_y);
  pass_filter3.setFilterFieldName("z");
  pass_filter3.setFilterLimits(z_min, z_max);
  pass_filter3.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter3.filter(*filtered_cloud_1);

  if (filtered_cloud_1->empty())
    return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // voxel : filtered_cloud_1 -> filtered_cloud_2

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(filtered_cloud_1);
  voxel_filter.setLeafSize(x_leaf, y_leaf, z_leaf);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*filtered_cloud_2);

  if (filtered_cloud_2->empty())
    return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // outliner : filtered_cloud_2 -> three_filtered_cloud

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliner_filter;
  outliner_filter.setInputCloud(filtered_cloud_2);
  outliner_filter.setMeanK(meanK);
  outliner_filter.setStddevMulThresh(threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr three_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  outliner_filter.filter(*three_filtered_cloud);

  if (three_filtered_cloud->empty())
    return;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // tf calculation : three_filtered_cloud -> transformed_cloud

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << x, y, z;
  transform.rotate(Eigen::AngleAxisf(deg_p, Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(deg_y, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(deg_r, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*filtered_cloud_2, *transformed_cloud, transform);

  if (transformed_cloud->empty())
    return;

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  sensor_msgs::PointCloud2 final;
  pcl::toROSMsg(*transformed_cloud, final);
  final.header.frame_id = "base_link";
  sensor_msgs::PointCloud2ConstPtr fiptr(new sensor_msgs::PointCloud2(final));
  if (marker_arg == true)
  {
    output_pub.publish(final);
  }
  max_Z(fiptr, flipper);
  return;
}

void max_Z(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, int flipper)
{
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  float max_z = -numeric_limits<float>::infinity(); // initialize to negative infinity
  float y_of_max_z = 0;
  float x_of_max_z = 0;

  for (const auto &point : cloud->points)
  {
    if (point.z > max_z)
    {
      max_z = point.z;
      y_of_max_z = point.y;
      x_of_max_z = point.x;
    }
  }
  float tan = max_z / y_of_max_z;
  float angle = atan(tan);
  angle = angle * 180 / M_PI;

  atan_data[flipper];
  flipper_xyz[flipper][0] = x_of_max_z;
  flipper_xyz[flipper][1] = y_of_max_z;
  flipper_xyz[flipper][2] = max_z;
  return;
}

void marker(int flipper, const ros::Publisher pub, const ros::Publisher textpub, float input_float[3])
{
  visualization_msgs::Marker line;
  line.header.frame_id = "base_link";
  line.header.stamp = ros::Time::now();
  line.ns = "line";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;
  line.scale.x = 0.01;
  geometry_msgs::Point p1, p2;
  if (flipper == FLIPPER_FL || flipper == FLIPPER_BL)
    p1.x = -0.1;
  else if (flipper == FLIPPER_FR || flipper == FLIPPER_BR)
    p1.x = 0.1;
  p1.y = 0;
  p1.z = 0;
  p2.x = input_float[0];
  p2.y = input_float[1];
  p2.z = input_float[2];
  line.points.push_back(p2);
  line.points.push_back(p1);
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;
  line.color.a = 1.0;
  pub.publish(line);

  visualization_msgs::Marker text;
  text.header.frame_id = "base_link";
  text.header.stamp = ros::Time::now();
  text.ns = "text_Z";
  text.id = 0;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.pose.position.x = input_float[0];
  text.pose.position.y = input_float[1];
  text.pose.position.z = input_float[2];
  text.scale.z = 0.1;
  text.color.a = 1.0;
  text.color.r = 0.0;
  text.color.g = 1.0;
  text.color.b = 0.0;

  text.text = std::to_string(input_float[2]);
  textpub.publish(text);
}

void flipper_back(float angle_L, float angle_R)
{
  if (auto_trigger[FLIPPER_BL] == false)
  {
    angle_L = 45;
  }
  if (auto_trigger[FLIPPER_BR] == false)
  {
    angle_R = 45;
  }

  if (angle_L > MAX_B_FLIPPER)
    angle_L = MAX_B_FLIPPER;
  if (angle_L < MIN_B_FLIPPER)
    angle_L = MIN_B_FLIPPER;

  if (angle_R > MAX_B_FLIPPER)
    angle_R = MAX_B_FLIPPER;
  if (angle_R < MIN_B_FLIPPER)
    angle_R = MIN_B_FLIPPER;

  target_angle[FLIPPER_BL] = -toRAD(angle_L);
  target_angle[FLIPPER_BR] = toRAD(angle_R);

  if (now_angle[FLIPPER_BL] > target_angle[FLIPPER_BL] + FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_BL] -= FLIPPER_SPEED_GAIN;
  else if (now_angle[FLIPPER_BL] < target_angle[FLIPPER_BL] - FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_FL] += FLIPPER_SPEED_GAIN;
  else if ((now_angle[FLIPPER_BL] > target_angle[FLIPPER_BL] - FLIPPER_SPEED_GAIN) && (now_angle[FLIPPER_BL] < target_angle[FLIPPER_BL] + FLIPPER_SPEED_GAIN))
    now_angle[FLIPPER_BL] = target_angle[FLIPPER_BL];

  if (now_angle[FLIPPER_BR] > target_angle[FLIPPER_BR] + FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_BR] -= FLIPPER_SPEED_GAIN;
  else if (now_angle[FLIPPER_BR] < target_angle[FLIPPER_BR] - FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_BR] += FLIPPER_SPEED_GAIN;
  else if ((now_angle[FLIPPER_BR] > target_angle[FLIPPER_BR] - FLIPPER_SPEED_GAIN) && (now_angle[FLIPPER_BR] < target_angle[FLIPPER_BR] + FLIPPER_SPEED_GAIN))
    now_angle[FLIPPER_BR] = target_angle[FLIPPER_BR];

  BACK_DATA.data = {now_angle[FLIPPER_BR], now_angle[FLIPPER_BL]};
  Back_angle.publish(BACK_DATA);
}

void flipper_front(float angle_L, float angle_R)
{
  if (auto_trigger[FLIPPER_FL] == false)
  {
    angle_L = 45;
  }
  if (auto_trigger[FLIPPER_FR] == false)
  {
    angle_R = 45;
  }

  if (angle_L > MAX_F_FLIPPER)
    angle_L = MAX_F_FLIPPER;
  if (angle_L < MIN_F_FLIPPER)
    angle_L = MIN_F_FLIPPER;
  if (angle_R > MAX_F_FLIPPER)
    angle_R = MAX_F_FLIPPER;
  if (angle_R < MIN_F_FLIPPER)
    angle_R = MIN_F_FLIPPER;
  target_angle[FLIPPER_FL] = toRAD(angle_L);
  target_angle[FLIPPER_FR] = -toRAD(angle_R);

  if (now_angle[FLIPPER_FL] > target_angle[FLIPPER_FL] + FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_FL] -= FLIPPER_SPEED_GAIN;
  else if (now_angle[FLIPPER_FL] < target_angle[FLIPPER_FL] - FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_FL] += FLIPPER_SPEED_GAIN;
  else if ((now_angle[FLIPPER_FL] > target_angle[FLIPPER_FL] - FLIPPER_SPEED_GAIN) && (now_angle[FLIPPER_FL] < target_angle[FLIPPER_FL] + FLIPPER_SPEED_GAIN))
    now_angle[FLIPPER_FL] = target_angle[FLIPPER_FL];

  if (now_angle[FLIPPER_FR] > target_angle[FLIPPER_FR] + FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_FR] -= FLIPPER_SPEED_GAIN;
  else if (now_angle[FLIPPER_FR] < target_angle[FLIPPER_FR] - FLIPPER_SPEED_GAIN)
    now_angle[FLIPPER_FR] += FLIPPER_SPEED_GAIN;
  else if ((now_angle[FLIPPER_FR] > target_angle[FLIPPER_FR] - FLIPPER_SPEED_GAIN) && (now_angle[FLIPPER_FR] < target_angle[FLIPPER_FR] + FLIPPER_SPEED_GAIN))
    now_angle[FLIPPER_FR] = target_angle[FLIPPER_FR];

  FRONT_DATA.data = {now_angle[FLIPPER_FL], now_angle[FLIPPER_FR]};
  Front_angle.publish(FRONT_DATA);
}

float MAF(float *input, int flipper)
{
  MAF_input[flipper][maf_index[flipper]] = input[flipper];
  maf_index[flipper]++;
  if (maf_index[flipper] >= MAF_MASK_SIZE)
    maf_index[flipper] = 0;
  float sum = 0;
  for (int i = 0; i < MAF_MASK_SIZE; i++)
    sum += MAF_input[flipper][i];
  return sum / MAF_MASK_SIZE;
}


