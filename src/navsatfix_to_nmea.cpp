#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>


ros::Publisher nmea_publisher;

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &message)
{
  std::stringstream nmea;
  nmea << "$GPGGA,";

  auto time_seconds = static_cast<time_t>(message->header.stamp.sec);
  
  tm time_struct;
  gmtime_r(&time_seconds, &time_struct);

  nmea << std::setfill('0') << std::setw(2) << time_struct.tm_hour;
  nmea << time_struct.tm_min << time_struct.tm_sec;

  int milliseconds = message->header.stamp.nsec / 1000000;

  nmea << "." << std::setfill('0') << std::setw(3) << milliseconds;

  int lat_degrees = std::abs(message->latitude);
  nmea << "," << std::setfill('0') << std::setw(2) << lat_degrees;

  float lat_minutes = 60.0*(std::abs(message->latitude) - lat_degrees);

  int lat_minutes_int = lat_minutes;
  nmea << std::setfill('0') << std::setw(2) << lat_minutes_int;
  
  int lat_micro_minutes = 1000000000*(lat_minutes-lat_minutes_int);
  nmea << "." << std::setfill('0') << std::setw(2) << lat_micro_minutes;

  nmea_msgs::Sentence sentence;
  sentence.header = message->header;
  sentence.sentence = nmea.str();

  ROS_INFO_STREAM(sentence);
  nmea_publisher.publish(sentence);

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navsatfix_to_nmea");

  ros::NodeHandle nh("~");

  nmea_publisher = nh.advertise<nmea_msgs::Sentence>("output", 10);

  ros::Subscriber nav_sat_fix_subscriber = nh.subscribe("input", 10, &navSatFixCallback);

  ros::spin();
  return 0;
}

