#include <string>
#include <memory>
#include <array>
#include <cstdio>                                   // snprintf
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "amr_interfaces/msg/ultrasonic_raw.hpp"
#include "amr_interfaces/msg/ultrasonic_zones.hpp"
#include "visualization_msgs/msg/marker_array.hpp"  // ← RViz markers

using std::placeholders::_1;
using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

/*
 * UltrasonicSafetyStop Node
 *
 * Subscribes to raw ultrasonic distances published by the ethernet bridge node.
 * Classifies each sensor reading into a zone (CLEAR or PROTECTIVE) based on
 * direction-specific thresholds, then publishes the result.
 *
 * Sensor index mapping (matches UltrasonicRaw.msg):
 *   Index 0 = Front Left   (Ultrasonic_1) → protective threshold: 250 mm
 *   Index 1 = Front Right  (Ultrasonic_2) → protective threshold: 250 mm
 *   Index 2 = Rear Right   (Ultrasonic_3) → protective threshold: 130 mm
 *   Index 3 = Rear Left    (Ultrasonic_4) → protective threshold: 130 mm
 *
 * Published topics:
 *   /ultrasonic_safety_stop  [std_msgs/Bool]
 *   /ultrasonic_zones        [amr_interfaces/UltrasonicZones]
 *   /ultrasonic_markers      [visualization_msgs/MarkerArray]   ← new
 *
 * RViz marker layout (per sensor):
 *   • SPHERE  – sits at the sensor origin; GREEN = CLEAR, RED = PROTECTIVE
 *   • ARROW   – extends from sensor origin along +X (sensor forward); length = measured distance
 *   • TEXT    – shows label + distance in mm above the sensor origin
 *
 * All markers use the sensor's own TF frame as frame_id so they follow the
 * robot automatically without any manual pose math here.
 */
class UltrasonicSafetyStop : public rclcpp::Node
{
public:
  UltrasonicSafetyStop()
  : Node("ultrasonic_safety_stop_node")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    declare_parameter<double>("front_protective_distance_mm", 250.0);
    declare_parameter<double>("rear_protective_distance_mm",  300.0);
    declare_parameter<std::string>("ultrasonic_raw_topic",         "ultrasonic_raw");
    declare_parameter<std::string>("ultrasonic_safety_stop_topic", "ultrasonic_safety_stop");
    declare_parameter<std::string>("ultrasonic_zones_topic",       "ultrasonic_zones");
    declare_parameter<std::string>("ultrasonic_markers_topic",     "ultrasonic_markers");

    front_threshold_mm_ = get_parameter("front_protective_distance_mm").as_double();
    rear_threshold_mm_  = get_parameter("rear_protective_distance_mm").as_double();

    const std::string raw_topic     = get_parameter("ultrasonic_raw_topic").as_string();
    const std::string stop_topic    = get_parameter("ultrasonic_safety_stop_topic").as_string();
    const std::string zones_topic   = get_parameter("ultrasonic_zones_topic").as_string();
    const std::string markers_topic = get_parameter("ultrasonic_markers_topic").as_string();

    RCLCPP_INFO(get_logger(),
      "UltrasonicSafetyStop started. Front threshold: %.1f mm | Rear threshold: %.1f mm",
      front_threshold_mm_, rear_threshold_mm_);

    // ── Subscriber ───────────────────────────────────────────────────────────
    ultrasonic_raw_sub_ = create_subscription<amr_interfaces::msg::UltrasonicRaw>(
      raw_topic, 10,
      std::bind(&UltrasonicSafetyStop::ultrasonicCallback, this, _1));

    // ── Publishers ───────────────────────────────────────────────────────────
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic, 10);
    zones_pub_       = create_publisher<amr_interfaces::msg::UltrasonicZones>(zones_topic, 10);
    marker_pub_      = create_publisher<MarkerArray>(markers_topic, 10);   // ← new

    prev_any_protective_ = false;
  }

private:
  // ── Constants ──────────────────────────────────────────────────────────────

  // TF frame IDs set by gz_frame_id in the URDF – must match exactly.
  static constexpr std::array<const char *, 5> kSensorFrames = {
    "Ultrasonic_1",   // index 0: Front Left
    "Ultrasonic_2",   // index 1: Front Right
    "Ultrasonic_3",   // index 2: Rear Right
    "Ultrasonic_4",    // index 3: Rear Left
    "Ultrasonic_5"    // index 4: Rear Middle
  };

  // Short label used in the TEXT marker
  static constexpr std::array<const char *, 5> kSensorLabels = {
    "FL", "FR", "RR", "RL", "RM"
  };

  // Marker ID blocks (3 markers × 5 sensors = ids 0-11)
  //   ids  0– 4 : spheres
  //   ids  5– 9 : arrows
  //   ids  10–14 : text
  static constexpr int kIdSphere = 0;
  static constexpr int kIdArrow  = 5;
  static constexpr int kIdText   = 10;

  // ── Member variables ───────────────────────────────────────────────────────
  double front_threshold_mm_;
  double rear_threshold_mm_;
  bool   prev_any_protective_;

  rclcpp::Subscription<amr_interfaces::msg::UltrasonicRaw>::SharedPtr ultrasonic_raw_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    safety_stop_pub_;
  rclcpp::Publisher<amr_interfaces::msg::UltrasonicZones>::SharedPtr  zones_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr                            marker_pub_;   // ← new

  // ── Callback ───────────────────────────────────────────────────────────────
  void ultrasonicCallback(const amr_interfaces::msg::UltrasonicRaw & msg)
  {
    const std::array<double, 5> thresholds = {
      front_threshold_mm_,   // index 0: Front Left
      front_threshold_mm_,   // index 1: Front Right
      rear_threshold_mm_,    // index 2: Rear Right
      rear_threshold_mm_,    // index 3: Rear Left
      rear_threshold_mm_     // index 4: Rear Middle
    };
  
    // ── Zone classification ─────────────────────────────────────────────────
    amr_interfaces::msg::UltrasonicZones zones_msg;
    zones_msg.header = msg.header;

    bool any_protective = false;

    for (size_t i = 0; i < 5; ++i) {
      const float dist = msg.distances_mm[i];
      zones_msg.distances_mm[i] = dist;

      // dist == 0.0 → no valid measurement from STM32 → treat as CLEAR.
      // Change to (dist <= threshold) to make it fail-safe instead.
      if (dist > 0.0f && dist <= static_cast<float>(thresholds[i])) {
        zones_msg.zones[i] = amr_interfaces::msg::UltrasonicZones::ZONE_PROTECTIVE;
        any_protective = true;
      } else {
        zones_msg.zones[i] = amr_interfaces::msg::UltrasonicZones::ZONE_CLEAR;
      }
    }

    zones_msg.robot_zone = any_protective
      ? amr_interfaces::msg::UltrasonicZones::ZONE_PROTECTIVE
      : amr_interfaces::msg::UltrasonicZones::ZONE_CLEAR;

    zones_pub_->publish(zones_msg);

    // ── Safety stop (publish only on state change) ──────────────────────────
    if (any_protective != prev_any_protective_) {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = any_protective;
      safety_stop_pub_->publish(stop_msg);
      prev_any_protective_ = any_protective;
    }

    // ── RViz markers ────────────────────────────────────────────────────────
    marker_pub_->publish(buildMarkers(msg.header.stamp, zones_msg));
  }

  // ── Marker builder ─────────────────────────────────────────────────────────
  //
  // Returns a MarkerArray with 3 markers per sensor (sphere, arrow, text).
  // All markers live in the sensor's own TF frame so no pose arithmetic needed.
  MarkerArray buildMarkers(
    const rclcpp::Time & stamp,
    const amr_interfaces::msg::UltrasonicZones & zones) const
  {
    MarkerArray array;
    array.markers.reserve(15);  // 3 × 5 sensors

    for (size_t i = 0; i < 5; ++i) {
      const bool protective =
        (zones.zones[i] == amr_interfaces::msg::UltrasonicZones::ZONE_PROTECTIVE);
      const float dist_m = zones.distances_mm[i] / 1000.0f;  // mm → m

      // Colour: RED if protective, GREEN if clear
      const float r = protective ? 1.0f : 0.0f;
      const float g = protective ? 0.0f : 1.0f;

      // ── 1. SPHERE at sensor origin ────────────────────────────────────────
      {
        Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = kSensorFrames[i];
        m.ns              = "ultrasonic_sensors";
        m.id              = kIdSphere + static_cast<int>(i);
        m.type            = Marker::SPHERE;
        m.action          = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.05;  // 5 cm diameter sphere
        m.color.r = r;  m.color.g = g;  m.color.b = 0.0f;  m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);  // auto-expire if no updates
        array.markers.push_back(m);
      }

      // ── 2. ARROW from sensor origin along +X (sensor forward) ─────────────
      //  scale.x = shaft length (= measured distance, clamped to ≥1 cm so
      //            the arrow is always visible even when dist_m is 0)
      //  scale.y = shaft diameter
      //  scale.z = arrow-head diameter
      {
        Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = kSensorFrames[i];
        m.ns              = "ultrasonic_beams";
        m.id              = kIdArrow + static_cast<int>(i);
        m.type            = Marker::ARROW;
        m.action          = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = (dist_m > 0.01f) ? static_cast<double>(dist_m) : 0.01;
        m.scale.y = 0.010;   // shaft  1 cm  diameter
        m.scale.z = 0.015;   // head   1.5 cm diameter
        m.color.r = r;  m.color.g = g;  m.color.b = 0.0f;  m.color.a = 0.85f;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        array.markers.push_back(m);
      }

      // ── 3. TEXT label above the sensor origin ─────────────────────────────
      {
        Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = kSensorFrames[i];
        m.ns              = "ultrasonic_text";
        m.id              = kIdText + static_cast<int>(i);
        m.type            = Marker::TEXT_VIEW_FACING;
        m.action          = Marker::ADD;
        m.pose.position.z    = 0.10;   // 10 cm above sensor origin
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.04;              // character height 4 cm
        m.color.r = m.color.g = m.color.b = 1.0f;  m.color.a = 1.0f;  // white

        char buf[32];
        if (zones.distances_mm[i] > 0.0f) {
          std::snprintf(buf, sizeof(buf), "%s: %.0f mm",
            kSensorLabels[i], static_cast<double>(zones.distances_mm[i]));
        } else {
          std::snprintf(buf, sizeof(buf), "%s: --", kSensorLabels[i]);
        }
        m.text    = buf;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        array.markers.push_back(m);
      }
    }

    return array;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSafetyStop>());
  rclcpp::shutdown();
  return 0;
}