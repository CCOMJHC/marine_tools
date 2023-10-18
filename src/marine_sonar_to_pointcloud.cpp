#include <ros/ros.h>
#include <marine_acoustic_msgs/RawSonarImage.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

#include "ping.h"

ros::Publisher pointcloud_publisher;
float detection_threshold = 0.0;

float last_increment = 0.0;

void sonarPingCallback(const marine_acoustic_msgs::RawSonarImage::ConstPtr &msg)
{
  if(!msg->image.data.empty() && msg->image.dtype == marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32)
  {
    marine_tools::Ping ping(*msg);

    pcl::PointCloud<pcl::PointXYZI> pc;
    pc.header.frame_id = msg->header.frame_id;
    pc.header.stamp = msg->header.stamp.toNSec()/1000;

    // float start_range = 0.5*msg->ping_info.sound_speed*msg->sample0/msg->sample_rate;
    // float range_increment = 0.5*msg->ping_info.sound_speed/msg->sample_rate;
    const auto& db_re_background = ping.valuesReBackground();
    for(int i = 0; i < db_re_background.size(); i++)
    {
      //float value = reinterpret_cast<const float*>(msg->image.data.data())[i];
      float value = db_re_background[i];
      if(value > detection_threshold)
      {
        //auto range = start_range+ i*range_increment;
        float range = i*ping.binSize();
        if (range >= 15)
        {
          pcl::PointXYZI p;
          p.x = 0.0;
          p.y = 0.0;
          p.z = range;
          p.intensity = value;
          pc.push_back(p);
        }
      }
    }
    pointcloud_publisher.publish(pc);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "marine_sonar_to_pointcloud");

  ros::NodeHandle nh, pnh("~");

  detection_threshold = pnh.param("detection_threshold", 0.0);

  ros::Subscriber radar_subscriber = nh.subscribe("sonar", 10, &sonarPingCallback);

  pointcloud_publisher = pnh.advertise<pcl::PointCloud<pcl::PointXYZI> >("pointcloud", 10);
    
  ros::spin();
  return 0;
}    
