#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sys/time.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "map.h"

using namespace std;


class EDGCloudPubNode
{
    public:
        EDGCloudPubNode();
        ~EDGCloudPubNode();

    private:
        ros::Subscriber scan_sub_;
        ros::Subscriber cloud_sub_;
        ros::Publisher  accumulate_scan_cloud_pub_;
        ros::Publisher  edgcloud_pub_;

        std::string target_frame_id_, fixed_frame_id_;

        tf::TransformListener tf_;
        laser_geometry::LaserProjection projector_;

        map_t* map_;
        int map_size_x_, map_size_y_;
        double cell_scale_, map_origin_x_, map_origin_y_;
        double edgthreshold_;

        ros::Duration cloud_pub_interval_;
        std::vector<sensor_msgs::PointCloud2> cloud2_v_;
        int cloud2_v_size_;

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

        sensor_msgs::PointCloud2 getScanCloud(const sensor_msgs::LaserScan& scan);
        sensor_msgs::PointCloud getEDGCloud();
};


EDGCloudPubNode::EDGCloudPubNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("map_size_x", map_size_x_, 600);
  private_nh.param("map_size_y", map_size_y_, 600);
  private_nh.param("cell_scale", cell_scale_, 0.10);
  private_nh.param("map_origin_x", map_origin_x_, 0.0);
  private_nh.param("map_origin_y", map_origin_y_, 0.0);
  private_nh.param("edgthreshold", edgthreshold_, 0.05);

  private_nh.param("target_frame_id", target_frame_id_, std::string("/base_link"));
  private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("/odom"));

  double cloud_pub_interval;
  private_nh.param("cloud_pub_interval", cloud_pub_interval, 0.05);
  cloud_pub_interval_.fromSec(cloud_pub_interval);

  private_nh.param("cloud2_v_size", cloud2_v_size_, 50);

  map_ = map_alloc();
  map_->size_x = map_size_x_;
  map_->size_y = map_size_y_;
  map_->scale = cell_scale_;
  map_->origin_x = map_origin_x_;
  map_->origin_y = map_origin_y_;
  map_->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map_->size_x*map_->size_y);
  for(int i = 0; i < map_->size_x * map_->size_y; i++) {
    map_->cells[i].min = 0.0;
    map_->cells[i].max = 0.0;
    map_->cells[i].sum_x = 0.0;
    map_->cells[i].sum_y = 0.0;
    map_->cells[i].visit = 0;
  }

  ros::NodeHandle nh;
  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &EDGCloudPubNode::scanCallback, this);
  cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(std::string("/hokuyo3d/hokuyo_cloud2"), 0, &EDGCloudPubNode::cloudCallback, this);
  edgcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/edgcloud", 100, false);
  accumulate_scan_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/accumulate_scan_cloud", 100, false);
}

EDGCloudPubNode::~EDGCloudPubNode()
{
    map_free(map_);
    map_ = NULL;

}

sensor_msgs::PointCloud2 EDGCloudPubNode::getScanCloud(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::PointCloud2 scan_cloud2;
  projector_.projectLaser(scan, scan_cloud2);

  return scan_cloud2;
}

sensor_msgs::PointCloud EDGCloudPubNode::getEDGCloud()
{
  sensor_msgs::PointCloud cloud_msg;
  cloud_msg.points.clear();
  for(int i = 0; i < map_->size_x; i++) {
    for(int j = 0; j < map_->size_y; j++) {
      int map_index = MAP_INDEX(map_,i,j);
      if(!(map_->cells[map_index].visit)) continue;
      double diff = map_->cells[map_index].max - map_->cells[map_index].min;
      if(edgthreshold_ < diff) {
        geometry_msgs::Point32 point;
        point.x = map_->cells[map_index].sum_x / map_->cells[map_index].visit;
        point.y = map_->cells[map_index].sum_y / map_->cells[map_index].visit;
        point.z = diff;
        cloud_msg.points.push_back(point);
      }
      map_->cells[map_index].min = 0.0;
      map_->cells[map_index].max = 0.0;
      map_->cells[map_index].sum_x = 0.0;
      map_->cells[map_index].sum_y = 0.0;
      map_->cells[map_index].visit = 0;
    }
  }
  
  return cloud_msg;
}

void EDGCloudPubNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  static ros::Time last_cloud_pub(0,0);

  cloud2_v_.push_back(getScanCloud(*scan));

  if (cloud2_v_.size() > cloud2_v_size_) {
    cloud2_v_.erase(cloud2_v_.begin());
  }

  if ((scan->header.stamp - last_cloud_pub) < cloud_pub_interval_) return;

  sensor_msgs::PointCloud2 tmp_cloud2;
  tmp_cloud2.data.clear();
  for (int i=0; i < cloud2_v_.size(); i++) {
    tf::StampedTransform transform;
    try {
      // tf_.waitForTransform(target_frame_id_, 
      //                      scan->header.stamp,
      //                      cloud2_v_.at(i).header.frame_id,
      //                      cloud2_v_.at(i).header.stamp,
      //                      fixed_frame_id_,
      //                      ros::Duration(0.5));

      tf_.lookupTransform(target_frame_id_,
                          scan->header.stamp,
                          cloud2_v_.at(i).header.frame_id,
                          cloud2_v_.at(i).header.stamp,
                          fixed_frame_id_,
                          transform);
    } catch(tf::TransformException e) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud2_v_.at(i), *pcl_cloud);
    pcl_ros::transformPointCloud(*pcl_cloud,
                                 *pcl_cloud,
                                 transform);
    for (int j=0; j < pcl_cloud->points.size(); j++) {
      map_updata_cell(map_, pcl_cloud->points.at(j).x, pcl_cloud->points.at(j).y, pcl_cloud->points.at(j).z);
    }
    sensor_msgs::PointCloud2 transformed_cloud2;
    toROSMsg (*pcl_cloud, transformed_cloud2);
    pcl::concatenatePointCloud(tmp_cloud2, transformed_cloud2, tmp_cloud2);
  }

  tmp_cloud2.header.frame_id = target_frame_id_;
  tmp_cloud2.header.stamp = scan->header.stamp;
  accumulate_scan_cloud_pub_.publish(tmp_cloud2);

  sensor_msgs::convertPointCloudToPointCloud2(getEDGCloud(), tmp_cloud2);
  tmp_cloud2.header.frame_id = target_frame_id_;
  tmp_cloud2.header.stamp = scan->header.stamp;
  edgcloud_pub_.publish(tmp_cloud2);

  last_cloud_pub = scan->header.stamp;
}

void EDGCloudPubNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  // ros::Time start = ros::Time::now();
  // static unsigned int step_num = 0;
  // static double add_time = 0.0;
  // static double add_size = 0.0;

  static ros::Time last_cloud_pub(cloud->header.stamp);

  if ((cloud->header.stamp - last_cloud_pub) < cloud_pub_interval_) return;

  sensor_msgs::PointCloud2 tmp_cloud2;
  try {
    pcl_ros::transformPointCloud(target_frame_id_, *cloud, tmp_cloud2, tf_);
  } catch(tf::TransformException e) {
    ROS_WARN("Failed transformPointCloud (%s)", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(tmp_cloud2, *pcl_cloud);
  for (int j=0; j<pcl_cloud->points.size(); j++) {
    map_updata_cell(map_, pcl_cloud->points.at(j).x, pcl_cloud->points.at(j).y, pcl_cloud->points.at(j).z);
  }

  sensor_msgs::convertPointCloudToPointCloud2(getEDGCloud(), tmp_cloud2);
  tmp_cloud2.header.frame_id = target_frame_id_;
  tmp_cloud2.header.stamp = cloud->header.stamp;
  edgcloud_pub_.publish(tmp_cloud2);

  last_cloud_pub = cloud->header.stamp;

  // double time = ros::Time::now().toSec() - start.toSec();
  // // ROS_INFO_STREAM("time: " << time << " ");
  // add_time += time;
  // step_num++;
  // ROS_INFO_STREAM("ave_time: " << (add_time/(double)step_num));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "edgcloud_pub_node");
    EDGCloudPubNode edg_cloud_pub;

    ros::spin();
    return 0;

}
