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


class AccumulateScanNode
{
    public:
        AccumulateScanNode();
        ~AccumulateScanNode();

    private:
        ros::Subscriber scan_sub_;
        ros::Subscriber cloud_sub_;
        ros::Publisher  accumulate_scan_cloud_pub_;
        ros::Publisher  elevation_difference_cloud_pub_;

        tf::TransformListener tf_;
        laser_geometry::LaserProjection projector_;

        map_t* map_;

        ros::Duration cloud_pub_interval_;
        std::vector<sensor_msgs::PointCloud2> cloud2_v_;

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        sensor_msgs::PointCloud2 getScanCloud(const sensor_msgs::LaserScan& scan);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};


AccumulateScanNode::AccumulateScanNode()
{
    map_ = map_alloc();
    map_->size_x = 600;
    map_->size_y = 600;
    map_->scale = 0.05;
    map_->origin_x = 0.0;
    map_->origin_y = 0.0;
    map_->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map_->size_x*map_->size_y);

    ros::NodeHandle nh;
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &AccumulateScanNode::scanCallback, this);
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(std::string("/hokuyo3d/hokuyo_cloud2"), 100, &AccumulateScanNode::cloudCallback, this);
    elevation_difference_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/elevation_difference_cloud", 100, false);
    accumulate_scan_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/accumulate_scan_cloud", 100, false);
    cloud_pub_interval_.fromSec(0.1);
}

AccumulateScanNode::~AccumulateScanNode()
{
    map_free(map_);
    map_ = NULL;

}

sensor_msgs::PointCloud2 AccumulateScanNode::getScanCloud(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::PointCloud2 scan_cloud2;
  projector_.projectLaser(scan, scan_cloud2);

  return scan_cloud2;
}

void AccumulateScanNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static ros::Time last_cloud_pub(0,0);

    cloud2_v_.push_back(getScanCloud(*scan));
    if ((scan->header.stamp - last_cloud_pub) > cloud_pub_interval_)
    {

      for(int i = 0; i < map_->size_x * map_->size_y; i++) {
          map_->cells[i].min = 0.0;
          map_->cells[i].max = 0.0;
          map_->cells[i].diff = 0.0;
      }

      sensor_msgs::PointCloud2 cloud2_msg;
      cloud2_msg.data.clear();
      for (int i=0; i < cloud2_v_.size(); i++) {
        tf::StampedTransform transform;
        if(!(tf_.waitForTransform(scan->header.frame_id, 
                                       scan->header.stamp,
                                       cloud2_v_.at(i).header.frame_id,
                                       cloud2_v_.at(i).header.stamp,
                                       "/odom",
                                       ros::Duration(0.5))))
        {
          last_cloud_pub = scan->header.stamp;
          cloud2_v_.clear();
          return;
        }
        // tf_.lookupTransform(scan->header.frame_id,
        tf_.lookupTransform("/base_link",
                                 scan->header.stamp,
                                 cloud2_v_.at(i).header.frame_id,
                                 cloud2_v_.at(i).header.stamp,
                                 "/odom",
                                 transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud2_v_.at(i), *pcl_cloud);
        pcl_ros::transformPointCloud(*pcl_cloud,
                                     *pcl_cloud,
                                     transform);
        for (int j = 0; j < pcl_cloud->points.size(); j++) {
            map_updata_cell(map_, pcl_cloud->points.at(j).x, pcl_cloud->points.at(j).y, pcl_cloud->points.at(j).z);
        }
        sensor_msgs::PointCloud2 transformed_cloud2;
        toROSMsg (*pcl_cloud, transformed_cloud2);
        pcl::concatenatePointCloud(cloud2_msg, transformed_cloud2, cloud2_msg);
      }

      // cloud2_msg.header.frame_id = scan->header.frame_id;
      cloud2_msg.header.frame_id = "base_link";
      cloud2_msg.header.stamp = scan->header.stamp;
      accumulate_scan_cloud_pub_.publish(cloud2_msg);

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl_cloud->points.clear();
      for(int i = 0; i < map_->size_x; i++) {
        for(int j = 0; j < map_->size_y; j++) {
          // if(0.03 > map_->cells[MAP_INDEX(map_, i, j)].diff) continue;
          if(!(map_->cells[MAP_INDEX(map_, i, j)].diff)) continue;
          pcl::PointXYZ pcl_point;
          pcl_point.x = MAP_WXGX(map_, i);
          pcl_point.y = MAP_WXGX(map_, j);
          pcl_point.z = map_->cells[MAP_INDEX(map_, i, j)].diff;
          pcl_cloud->points.push_back(pcl_point);
        }
      }
      cloud2_msg.data.clear();
      toROSMsg (*pcl_cloud, cloud2_msg);

      // cloud2_msg.header.frame_id = scan->header.frame_id;
      cloud2_msg.header.frame_id = "base_link";
      cloud2_msg.header.stamp = scan->header.stamp;
      elevation_difference_cloud_pub_.publish(cloud2_msg);
      last_cloud_pub = scan->header.stamp;
    }
    if (cloud2_v_.size() > 50) {
      cloud2_v_.erase(cloud2_v_.begin());
    }
}

void AccumulateScanNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    static ros::Time last_cloud_pub(0,0);

    // if ((cloud->header.stamp - last_cloud_pub) > cloud_pub_interval_)
    // {
      for(int i = 0; i < map_->size_x * map_->size_y; i++) {
          map_->cells[i].min = 0.0;
          map_->cells[i].max = 0.0;
          map_->cells[i].diff = 0.0;
      }

      sensor_msgs::PointCloud2 transformed_cloud2;
      if(!pcl_ros::transformPointCloud("base_link", *cloud, transformed_cloud2, tf_)) {
        last_cloud_pub = cloud->header.stamp;
        return;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(transformed_cloud2, *pcl_cloud);
      for (int j = 0; j < pcl_cloud->points.size(); j++) {
          map_updata_cell(map_, pcl_cloud->points.at(j).x, pcl_cloud->points.at(j).y, pcl_cloud->points.at(j).z);
      }

      
      sensor_msgs::PointCloud cloud_msg;
      cloud_msg.points.clear();
      for(int i = 0; i < map_->size_x; i++) {
        for(int j = 0; j < map_->size_y; j++) {
          // if(0.03 > map_->cells[MAP_INDEX(map_, i, j)].diff) continue;
          if(!(map_->cells[MAP_INDEX(map_, i, j)].diff)) continue;
          geometry_msgs::Point32 point;
          point.x = MAP_WXGX(map_, i);
          point.y = MAP_WXGX(map_, j);
          point.z = map_->cells[MAP_INDEX(map_, i, j)].diff;
          cloud_msg.points.push_back(point);
        }
      }

      sensor_msgs::PointCloud2 cloud2_msg;
      cloud2_msg.data.clear();
      sensor_msgs::convertPointCloudToPointCloud2(cloud_msg, cloud2_msg);
      // cloud2_msg.header.frame_id = scan->header.frame_id;
      cloud2_msg.header.frame_id = "base_link";
      cloud2_msg.header.stamp = cloud->header.stamp;
      elevation_difference_cloud_pub_.publish(cloud2_msg);
      last_cloud_pub = cloud->header.stamp;
    // }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "making_envir_cloud");
    AccumulateScanNode asn;

    ros::spin();
    return 0;

}
