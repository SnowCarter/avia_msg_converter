#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pubLaserCloudRaw;
void laserCloudHandler(const livox_ros_driver::CustomMsgConstPtr& msg)
{
    PointCloudXYZI pl_full;
    int plsize = msg->point_num;
    pl_full.resize(plsize);
    uint valid_num = 0;

    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < 6) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % 3 == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

        }
      }
    
    // pubLaserCloudRaw.publish(laserCloudOut);
    }
    // publish p_pre->pl_full through pubLaserCloudRaw
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(pl_full, laserCloudmsg);
    laserCloudmsg.header.stamp = msg->header.stamp;
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudRaw.publish(laserCloudmsg);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "converter_node");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe("/livox/lidar", 200000, laserCloudHandler);
    pubLaserCloudRaw = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidarPointCloud", 100000);
    ros::spin();
    return 0;


}