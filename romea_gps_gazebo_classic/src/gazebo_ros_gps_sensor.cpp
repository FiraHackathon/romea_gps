// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <iostream>
#include <memory>
#include <string>

// gazebo
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/utils.hpp"

// ros
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"

// romea core
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_gps/nmea/GGAFrame.hpp"
#include "romea_core_gps/nmea/RMCFrame.hpp"
#include "romea_core_gps/nmea/HDTFrame.hpp"

// local
#include "romea_gps_gazebo/gazebo_ros_gps_sensor.hpp"

namespace
{

const double DEFAULT_DHOP = 1;
const uint8_t DEFAUlT_FIX_STATUS = 0;
const uint16_t DEFAULT_SERVICE = 15;
const gazebo::common::Time GGA_STAMP_OFFSET = gazebo::common::Time(0, 0);
const gazebo::common::Time RMC_STAMP_OFFSET = gazebo::common::Time(0, 5000000);
const gazebo::common::Time HDT_STAMP_OFFSET = gazebo::common::Time(0, 5000000);

bool isPointInPolygon(const Eigen::Vector2d & point, const std::vector<Eigen::Vector2d> & polygon)
{
  int n = polygon.size();
  bool inside = false;

  for (int i = 0; i < n; i++) {
    Eigen::Vector2d p1 = polygon[i];
    Eigen::Vector2d p2 = polygon[(i + 1) % n];     // Le prochain point, boucle au premier

    // Vérifier si le point est sur un des sommets
    if ((p1.y() == point.y() && p1.x() == point.x()) ||
      (p2.y() == point.y() && p2.x() == point.x()))
    {
      return true;       // Le point est exactement sur un sommet
    }

    // Vérifier si le segment croise l'axe horizontal passant par le point
    if ((p1.y() > point.y()) != (p2.y() > point.y())) {
      // Calculer la coordonnée x du point d'intersection du segment avec l'axe horizontal
      double intersectX = (p2.x() - p1.x()) * (point.y() - p1.y()) / (p2.y() - p1.y()) + p1.x();
      if (point.x() < intersectX) {
        inside = !inside;         // Inverser l'état d'intérieur
      }
    }
  }
  return inside;
}

bool isPointInAPolygon(
  const Eigen::Vector2d & point,
  const std::vector<std::vector<Eigen::Vector2d>> & polygons)
{
  for (const auto & polygon:polygons) {
    if (isPointInPolygon(point, polygon)) {
      return true;
    }
  }
  return false;
}

}  // namespace

namespace romea
{
namespace ros2
{

class GazeboRosGpsSensorPrivate
{
public:
  /// enable dual_antenna
  bool dual_antenna_{false};

  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publish for gps message
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_sentence_pub_;

  /// GPS message modified each update
  sensor_msgs::msg::NavSatFix::SharedPtr fix_msg_;
  geometry_msgs::msg::TwistStamped::SharedPtr vel_msg_;
  nmea_msgs::msg::Sentence::SharedPtr nmea_gga_sentence_msg_;
  nmea_msgs::msg::Sentence::SharedPtr nmea_rmc_sentence_msg_;
  nmea_msgs::msg::Sentence::SharedPtr nmea_hdt_sentence_msg_;

  /// GPS sensor this plugin is attached to
  gazebo::sensors::GpsSensorPtr sensor_;
  /// Parent link entity
  gazebo::physics::EntityPtr sensor_parent_link_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// sheds
  std::vector<std::vector<Eigen::Vector2d>> sheds_;

  /// Publish latest gps data to ROS
  void OnUpdate();
};

GazeboRosGpsSensor::GazeboRosGpsSensor()
: impl_(std::make_unique<GazeboRosGpsSensorPrivate>())
{
}

GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
}

void GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  //  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sheds_.resize(2);
  impl_->sheds_[0].push_back(Eigen::Vector2d(3.434608953, 46.34009787));
  impl_->sheds_[0].push_back(Eigen::Vector2d(3.434476524, 46.34034087));
  impl_->sheds_[0].push_back(Eigen::Vector2d(3.434091943, 46.34024158));
  impl_->sheds_[0].push_back(Eigen::Vector2d(3.434220698, 46.33999756));
  impl_->sheds_[1].push_back(Eigen::Vector2d(3.434968908, 46.34022093));
  impl_->sheds_[1].push_back(Eigen::Vector2d(3.435758558, 46.34042847));
  impl_->sheds_[1].push_back(Eigen::Vector2d(3.435642186, 46.34064096));
  impl_->sheds_[1].push_back(Eigen::Vector2d(3.434852482, 46.34043343));

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a GPS sensor. Exiting.");
    return;
  }

  impl_->sensor_parent_link_ = gazebo::physics::get_world()->EntityByName(
    impl_->sensor_->ParentName());

  unsigned int fix_status = DEFAUlT_FIX_STATUS;
  if (_sdf->HasElement("status")) {
    _sdf->GetElement("status")->GetValue()->Get(fix_status);
  }

  unsigned int service = DEFAULT_SERVICE;
  if (_sdf->HasElement("service")) {
    _sdf->GetElement("service")->GetValue()->Get(service);
  }

  if (_sdf->HasElement("dual_antenna")) {
    _sdf->GetElement("dual_antenna")->GetValue()->Get(impl_->dual_antenna_);
  }

  //  Init fix publisher
  impl_->fix_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "fix", qos.get_publisher_qos("fix", rclcpp::SensorDataQoS().reliable()));

  // Init fix msg
  impl_->fix_msg_ = std::make_shared<sensor_msgs::msg::NavSatFix>();
  impl_->fix_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  using SNT = gazebo::sensors::SensorNoiseType;
  impl_->fix_msg_->position_covariance[0] = gazebo_ros::NoiseVariance(
    impl_->sensor_->Noise(SNT::GPS_POSITION_LATITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance[4] = gazebo_ros::NoiseVariance(
    impl_->sensor_->Noise(SNT::GPS_POSITION_LONGITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance[8] = gazebo_ros::NoiseVariance(
    impl_->sensor_->Noise(SNT::GPS_POSITION_ALTITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  impl_->fix_msg_->status.status =
    static_cast<sensor_msgs::msg::NavSatStatus::_status_type>(fix_status);
  impl_->fix_msg_->status.service =
    static_cast<sensor_msgs::msg::NavSatStatus::_service_type>(service);


  //  Init vel publisher
  impl_->vel_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "vel", qos.get_publisher_qos("vel", rclcpp::SensorDataQoS().reliable()));

  // Init vel message
  impl_->vel_msg_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  impl_->vel_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  //  Init nmea sentence publisher
  impl_->nmea_sentence_pub_ = impl_->ros_node_->create_publisher<nmea_msgs::msg::Sentence>(
    "nmea_sentence", qos.get_publisher_qos("nmea_sentence", rclcpp::SensorDataQoS().reliable()));

  // Init nmea messages
  impl_->nmea_gga_sentence_msg_ = std::make_shared<nmea_msgs::msg::Sentence>();
  impl_->nmea_gga_sentence_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->nmea_rmc_sentence_msg_ = std::make_shared<nmea_msgs::msg::Sentence>();
  impl_->nmea_rmc_sentence_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->nmea_hdt_sentence_msg_ = std::make_shared<nmea_msgs::msg::Sentence>();
  impl_->nmea_hdt_sentence_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosGpsSensorPrivate::OnUpdate()
{
  auto sensor_stamp = sensor_->LastUpdateTime();

  auto gga_stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_stamp + GGA_STAMP_OFFSET);

  double latitude = sensor_->Latitude().Degree();
  double longitude = sensor_->Longitude().Degree();
  double altitude = sensor_->Altitude();
  double v_east = sensor_->VelocityEast();
  double v_north = sensor_->VelocityNorth();
  double v_up = sensor_->VelocityUp();

  if (!isPointInAPolygon(Eigen::Vector2d(longitude, latitude), sheds_)) {

    core::GGAFrame gga_frame;
    gga_frame.fixTime = core::FixTime(gga_stamp.sec, gga_stamp.nanosec);
    gga_frame.talkerId = core::TalkerId::GP;
    gga_frame.latitude = core::Latitude(latitude / 180. * M_PI);
    gga_frame.longitude = core::Longitude(longitude / 180. * M_PI);
    gga_frame.altitudeAboveGeoid = altitude;
    gga_frame.fixQuality = core::FixQuality::SIMULATION_FIX;
    gga_frame.geoidHeight = 0;
    gga_frame.horizontalDilutionOfPrecision = DEFAULT_DHOP;
    gga_frame.numberSatellitesUsedToComputeFix = 0;
    nmea_gga_sentence_msg_->sentence = gga_frame.toNMEA();
    nmea_gga_sentence_msg_->header.stamp = gga_stamp;
    nmea_sentence_pub_->publish(*nmea_gga_sentence_msg_);

    auto rmc_stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
      sensor_stamp + RMC_STAMP_OFFSET);

    core::RMCFrame rmc_frame;
    rmc_frame.fixTime = core::FixTime(rmc_stamp.sec, rmc_stamp.nanosec);
    rmc_frame.status = core::RMCFrame::Status::Void;
    rmc_frame.talkerId = core::TalkerId::GP;
    rmc_frame.latitude = core::Latitude(latitude / 180. * M_PI);
    rmc_frame.longitude = core::Longitude(longitude / 180. * M_PI);
    rmc_frame.trackAngleTrue = M_PI_2 - std::atan2(v_north, v_east);
    rmc_frame.speedOverGroundInMeterPerSecond = std::sqrt(v_east * v_east + v_north * v_north);
    rmc_frame.fixQuality = core::FixQuality::SIMULATION_FIX;
    nmea_rmc_sentence_msg_->sentence = rmc_frame.toNMEA();
    nmea_rmc_sentence_msg_->header.stamp = rmc_stamp;
    nmea_sentence_pub_->publish(*nmea_rmc_sentence_msg_);

    if (dual_antenna_) {
      auto hdt_stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
        sensor_stamp + HDT_STAMP_OFFSET);

      core::HDTFrame hdt_frame;
      hdt_frame.talkerId = core::TalkerId::GP;
      hdt_frame.heading = core::between0And2Pi(
        M_PI_2 - (sensor_->Pose() + sensor_parent_link_->WorldPose()).Yaw());
      hdt_frame.trueNorth = true;
      nmea_hdt_sentence_msg_->header.stamp = hdt_stamp;
      nmea_hdt_sentence_msg_->sentence = hdt_frame.toNMEA();
      nmea_sentence_pub_->publish(*nmea_hdt_sentence_msg_);
    }

    fix_msg_->header.stamp = gga_stamp;
    fix_msg_->latitude = latitude;
    fix_msg_->longitude = longitude;
    fix_msg_->altitude = altitude;
    fix_pub_->publish(*fix_msg_);

    vel_msg_->header.stamp = rmc_stamp;
    vel_msg_->twist.linear.x = v_east;
    vel_msg_->twist.linear.y = v_north;
    vel_msg_->twist.linear.z = v_up;
    vel_pub_->publish(*vel_msg_);
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor)

}  // namespace ros2
}  // namespace romea
