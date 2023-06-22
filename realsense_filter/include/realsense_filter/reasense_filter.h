#include <ros/ros.h>
#include <ros/node_handle.h>

#include "angle.h"

#include <boost/make_shared.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <vector>

#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>

#define FLIPPER_SPEED_GAIN 0.05
#define MAX_F_FLIPPER 130
#define MIN_F_FLIPPER -90
#define MAX_B_FLIPPER 130
#define MIN_B_FLIPPER -90

#define FLIPPER_FL 0
#define FLIPPER_FR 1
#define FLIPPER_BL 2
#define FLIPPER_BR 3
#define ROLL 4
#define PITCH 5

#define MAF_MASK_SIZE 30
#define IMU_DATA_RELIANCE 0.98
#define ANGLE_POS_SUM 20
#define ANGLE_POS_SUM_HALF 10

#define AUTO_FLIPPER_TRIGGER 30

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointT> PointCloudT;

void frontthreadFunction(int argc, char **argv);

void backthreadFunction(int argc, char **argv);

void front_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);

void back_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);

void imu_callback(const sensor_msgs::Imu input_imu);

void three_filter(int flipper, const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg, const ros::Publisher output_pub,
                  float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                  float x_leaf, float y_leaf, float z_leaf, int meanK, float threshold,
                  float x, float y, float z, float deg_p, float deg_y, float deg_r);

void max_Z(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, int flipper);

void marker(int flipper, const ros::Publisher pub, const ros::Publisher textpub, float input_float[3]);

void flipper_front(float angle_L, float angle_R);

void flipper_back(float angle_L, float angle_R);

float MAF(float input[], int flipper);

void auto_flipper_trigger(int flipper1, int flipper2);

ros::Subscriber front_cloud_sub;
ros::Subscriber back_cloud_sub;

ros::Subscriber imu_sub;

ros::Publisher FL_point_pub;
ros::Publisher FR_point_pub;
ros::Publisher FL_marker;
ros::Publisher FR_marker;

ros::Publisher BL_point_pub;
ros::Publisher BR_point_pub;
ros::Publisher BL_marker;
ros::Publisher BR_marker;

ros::Publisher FL_marker_text;
ros::Publisher FR_marker_text;
ros::Publisher BL_marker_text;
ros::Publisher BR_marker_text;

ros::Publisher Front_angle;
ros::Publisher Back_angle;

std_msgs::Float64MultiArray FRONT_DATA;
std_msgs::Float64MultiArray BACK_DATA;

std_msgs::Float64 FL;
std_msgs::Float64 FR;
std_msgs::Float64 BL;
std_msgs::Float64 BR;

bool marker_arg = false;
bool front_arg = true;
bool back_arg = true;
bool init_arg = true;

float flipper_xyz[4][3];

float atan_data[4] = {
    0,
};

float now_angle[4] = {
    0,
};

float target_angle[4] = {
    0,
};

int max_z_cnt[4] = {
    0,
};

bool auto_trigger[4] = {false, false, false, false};

float MAF_input[6][MAF_MASK_SIZE];
int maf_index[6] = {
    0,
};

float imu_roll = 0;
float imu_pitch = 0;
float imu_yaw = 0;
