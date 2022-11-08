#include "romea_gps_utils/gps_data_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const std::string & raw_sentence,
                nmea_msgs::msg::Sentence & msg)
{
  msg.header.stamp =stamp ;
  msg.header.frame_id = frame_id;
  msg.sentence= raw_sentence;
}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const GGAFrame & gga_frame,
                sensor_msgs::msg::NavSatFix & msg)
{

  msg.header.stamp =stamp ;
  msg.header.frame_id = frame_id;

  msg.latitude= gga_frame.latitude->toDouble()*180./M_PI ;
  msg.longitude= gga_frame.longitude->toDouble()*180./M_PI;
  msg.altitude = gga_frame.altitudeAboveGeoid.value() + gga_frame.geoidHeight.value();

  double hdop = gga_frame.horizontalDilutionOfPrecision.value();
  msg.position_covariance[0] = hdop*hdop;
  msg.position_covariance[4] = hdop*hdop;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;


  FixQuality fix_quality =  gga_frame.fixQuality.value();
  if(fix_quality == FixQuality::INVALID_FIX)
  {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  else if (fix_quality==FixQuality::GPS_FIX)
  {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  else if(fix_quality==FixQuality::DGPS_FIX)
  {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  }
  else if(fix_quality==FixQuality::FLOAT_RTK_FIX || fix_quality==FixQuality::RTK_FIX)
  {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  }

  TalkerId talker_id = gga_frame.talkerId;

  if(talker_id == TalkerId::GP)
  {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
  else if(talker_id == TalkerId::GL)
  {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  }
  else if(talker_id == TalkerId::GA)
  {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  }
  else if(talker_id == TalkerId::GB || talker_id==TalkerId::BD)
  {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }

}

//-----------------------------------------------------------------------------
void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const RMCFrame & rmc_frame,
                geometry_msgs::msg::TwistStamped & msg)
{
  msg.header.stamp = stamp ;
  msg.header.frame_id = frame_id;

  msg.twist.linear.x = rmc_frame.speedOverGroundInMeterPerSecond.value() *
      std::sin(rmc_frame.trackAngleTrue.value());
  msg.twist.linear.y = rmc_frame.speedOverGroundInMeterPerSecond.value() *
      std::cos(rmc_frame.trackAngleTrue.value());
}

}
