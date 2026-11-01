#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace roka_terrain_mapping
{

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

inline double wrap_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct OrientedExtents2
{
  double length{0.0};
  double width{0.0};
};

inline Point2 operator-(const Point2 & left, const Point2 & right)
{
  return Point2{left.x - right.x, left.y - right.y};
}

inline Point2 operator/(const Point2 & point, double scale)
{
  return Point2{point.x / scale, point.y / scale};
}

inline double norm(const Point2 & point)
{
  return std::hypot(point.x, point.y);
}

inline double distance(const Point2 & left, const Point2 & right)
{
  return std::hypot(left.x - right.x, left.y - right.y);
}

// Shortest horizontal distance from a LiDAR point to the rectangular robot
// footprint.  A point on or inside the footprint has zero clearance.  This is
// intentionally measured from the body shell, not from the LiDAR origin or a
// target cluster centroid.
inline double rectangular_footprint_clearance(
  const Point2 & point, double front, double rear, double half_width)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
    !std::isfinite(front) || !std::isfinite(rear) ||
    !std::isfinite(half_width) || front < 0.0 || rear < 0.0 || half_width < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = point.x > front ? point.x - front :
    (point.x < -rear ? -rear - point.x : 0.0);
  const double dy = std::max(0.0, std::abs(point.y) - half_width);
  return std::hypot(dx, dy);
}

// Clearance at which a zero command must start.  The latency term covers the
// sensor/control pipeline and v^2/(2a) covers the configured physical braking
// ramp.  final_clearance is the distance remaining after the robot stops.
inline double required_stopping_clearance(
  double speed, double final_clearance, double pipeline_latency,
  double deceleration)
{
  if (!std::isfinite(speed) || !std::isfinite(final_clearance) ||
    !std::isfinite(pipeline_latency) || !std::isfinite(deceleration) ||
    final_clearance < 0.0 || pipeline_latency < 0.0 || deceleration <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  speed = std::abs(speed);
  return final_clearance + speed * pipeline_latency +
         speed * speed / (2.0 * deceleration);
}

// PCA-aligned horizontal dimensions.  Axis-aligned x/y bounds make a long,
// narrow robot dog appear wider while it turns; using its dominant direction
// keeps the 0.60 m width limit meaningful through a U-turn.
inline OrientedExtents2 oriented_horizontal_extents(
  const std::vector<Point2> & points)
{
  if (points.empty()) {
    return {};
  }
  Point2 mean;
  for (const auto & point : points) {
    mean.x += point.x;
    mean.y += point.y;
  }
  mean.x /= static_cast<double>(points.size());
  mean.y /= static_cast<double>(points.size());
  double cxx = 0.0;
  double cyy = 0.0;
  double cxy = 0.0;
  for (const auto & point : points) {
    const double dx = point.x - mean.x;
    const double dy = point.y - mean.y;
    cxx += dx * dx;
    cyy += dy * dy;
    cxy += dx * dy;
  }
  const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  double minimum_u = std::numeric_limits<double>::infinity();
  double maximum_u = -std::numeric_limits<double>::infinity();
  double minimum_v = std::numeric_limits<double>::infinity();
  double maximum_v = -std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    const double dx = point.x - mean.x;
    const double dy = point.y - mean.y;
    const double u = cosine * dx + sine * dy;
    const double v = -sine * dx + cosine * dy;
    minimum_u = std::min(minimum_u, u);
    maximum_u = std::max(maximum_u, u);
    minimum_v = std::min(minimum_v, v);
    maximum_v = std::max(maximum_v, v);
  }
  const double first = maximum_u - minimum_u;
  const double second = maximum_v - minimum_v;
  return OrientedExtents2{std::max(first, second), std::min(first, second)};
}

inline bool oriented_target_size_is_valid(
  const OrientedExtents2 & horizontal, double height, double minimum_extent,
  double maximum_length, double maximum_width, double maximum_height)
{
  constexpr double numerical_tolerance = 1.0e-9;
  return std::isfinite(horizontal.length) && std::isfinite(horizontal.width) &&
    std::isfinite(height) && horizontal.length + numerical_tolerance >= minimum_extent &&
    horizontal.length <= maximum_length + numerical_tolerance &&
    horizontal.width + numerical_tolerance >= minimum_extent &&
    horizontal.width <= maximum_width + numerical_tolerance &&
    height + numerical_tolerance >= minimum_extent &&
    height <= maximum_height + numerical_tolerance;
}

class ConstantVelocityTracker
{
public:
  void reset()
  {
    initialized_ = false;
    x_axis_ = Axis{};
    y_axis_ = Axis{};
  }

  void initialize(
    const Point2 & position, double position_variance = 0.10,
    double velocity_variance = 1.0, const Point2 & velocity = Point2{})
  {
    x_axis_ = Axis{
      position.x, velocity.x, position_variance, 0.0, 0.0, velocity_variance};
    y_axis_ = Axis{
      position.y, velocity.y, position_variance, 0.0, 0.0, velocity_variance};
    initialized_ = true;
  }

  bool initialized() const {return initialized_;}

  void predict(double dt, double acceleration_variance)
  {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0) {
      return;
    }
    dt = clamp(dt, 0.001, 0.50);
    predict_axis(x_axis_, dt, acceleration_variance);
    predict_axis(y_axis_, dt, acceleration_variance);
  }

  void update(const Point2 & measurement, double measurement_variance)
  {
    if (!initialized_) {
      initialize(measurement);
      return;
    }
    update_axis(x_axis_, measurement.x, measurement_variance);
    update_axis(y_axis_, measurement.y, measurement_variance);
  }

  Point2 position() const {return Point2{x_axis_.position, y_axis_.position};}
  Point2 velocity() const {return Point2{x_axis_.velocity, y_axis_.velocity};}
  double speed() const {return std::hypot(x_axis_.velocity, y_axis_.velocity);}

  void damp_velocity(double factor)
  {
    if (!initialized_ || !std::isfinite(factor)) {
      return;
    }
    factor = clamp(factor, 0.0, 1.0);
    x_axis_.velocity *= factor;
    y_axis_.velocity *= factor;
  }

  double innovation_distance_squared(
    const Point2 & measurement, double measurement_variance) const
  {
    if (!initialized_) {
      return 0.0;
    }
    const double x_variance = std::max(1e-9, x_axis_.p00 + measurement_variance);
    const double y_variance = std::max(1e-9, y_axis_.p00 + measurement_variance);
    const double dx = measurement.x - x_axis_.position;
    const double dy = measurement.y - y_axis_.position;
    return dx * dx / x_variance + dy * dy / y_variance;
  }

  double position_uncertainty() const
  {
    return std::sqrt(std::max(0.0, x_axis_.p00 + y_axis_.p00));
  }

private:
  struct Axis
  {
    double position{0.0};
    double velocity{0.0};
    double p00{1.0};
    double p01{0.0};
    double p10{0.0};
    double p11{1.0};
  };

  static void predict_axis(Axis & axis, double dt, double acceleration_variance)
  {
    axis.position += dt * axis.velocity;
    const double old_p00 = axis.p00;
    const double old_p01 = axis.p01;
    const double old_p10 = axis.p10;
    const double old_p11 = axis.p11;
    const double q = std::max(0.0, acceleration_variance);
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    axis.p00 = old_p00 + dt * (old_p10 + old_p01) + dt2 * old_p11 + 0.25 * dt4 * q;
    axis.p01 = old_p01 + dt * old_p11 + 0.5 * dt3 * q;
    axis.p10 = old_p10 + dt * old_p11 + 0.5 * dt3 * q;
    axis.p11 = old_p11 + dt2 * q;
  }

  static void update_axis(Axis & axis, double measurement, double measurement_variance)
  {
    const double variance = std::max(1e-9, measurement_variance);
    const double innovation = measurement - axis.position;
    const double innovation_variance = std::max(1e-9, axis.p00 + variance);
    const double k_position = axis.p00 / innovation_variance;
    const double k_velocity = axis.p10 / innovation_variance;
    const double old_p00 = axis.p00;
    const double old_p01 = axis.p01;
    const double old_p10 = axis.p10;
    const double old_p11 = axis.p11;
    axis.position += k_position * innovation;
    axis.velocity += k_velocity * innovation;
    axis.p00 = (1.0 - k_position) * old_p00;
    axis.p01 = (1.0 - k_position) * old_p01;
    axis.p10 = old_p10 - k_velocity * old_p00;
    axis.p11 = old_p11 - k_velocity * old_p01;
    const double cross = 0.5 * (axis.p01 + axis.p10);
    axis.p01 = cross;
    axis.p10 = cross;
  }

  bool initialized_{false};
  Axis x_axis_;
  Axis y_axis_;
};

struct TargetAssociationLimits
{
  double maximum_observed_speed{2.0};
  double maximum_speed_increase_acceleration{4.0};
  double direction_check_minimum_speed{0.15};
  double minimum_direction_allowance_rad{30.0 * M_PI / 180.0};
  double maximum_direction_rate_radps{120.0 * M_PI / 180.0};
};

inline bool kinematic_candidate_is_consistent(
  const Point2 & previous_position, const Point2 & candidate_position, double dt,
  const Point2 & predicted_velocity, bool reversal_unlocked,
  const TargetAssociationLimits & limits)
{
  if (!std::isfinite(dt) || dt <= 0.02) {
    return false;
  }
  const Point2 observed_velocity = (candidate_position - previous_position) / dt;
  const double observed_speed = norm(observed_velocity);
  const double predicted_speed = norm(predicted_velocity);
  if (!std::isfinite(observed_speed) || observed_speed > limits.maximum_observed_speed) {
    return false;
  }
  if (observed_speed - predicted_speed >
    limits.maximum_speed_increase_acceleration * dt + 0.20)
  {
    return false;
  }
  if (reversal_unlocked || observed_speed < limits.direction_check_minimum_speed ||
    predicted_speed < limits.direction_check_minimum_speed)
  {
    return true;
  }
  const double cosine = clamp(
    (observed_velocity.x * predicted_velocity.x +
    observed_velocity.y * predicted_velocity.y) /
    std::max(1.0e-9, observed_speed * predicted_speed), -1.0, 1.0);
  const double direction_change = std::acos(cosine);
  const double allowance = std::min(
    M_PI, std::max(
      limits.minimum_direction_allowance_rad,
      limits.maximum_direction_rate_radps * dt));
  return direction_change <= allowance;
}

inline bool shape_ratio_is_plausible(
  double candidate, double reference, double minimum_ratio = 0.40,
  double maximum_ratio = 2.50)
{
  if (!std::isfinite(candidate) || !std::isfinite(reference) ||
    candidate <= 0.0 || reference <= 0.0)
  {
    return false;
  }
  const double ratio = candidate / reference;
  return ratio >= minimum_ratio && ratio <= maximum_ratio;
}

struct TrailPoint
{
  Point2 position;
  double station{0.0};
};

class LeaderTrail
{
public:
  void clear() {points_.clear();}
  bool empty() const {return points_.empty();}
  std::size_t size() const {return points_.size();}
  double end_station() const {return points_.empty() ? 0.0 : points_.back().station;}
  const std::vector<TrailPoint> & points() const {return points_;}

  void truncate_after(double station)
  {
    if (points_.empty() || !std::isfinite(station)) {
      return;
    }
    while (points_.size() > 1U && points_.back().station > station + 1.0e-9) {
      points_.pop_back();
    }
  }

  void seed(const Point2 & robot, const Point2 & leader, double spacing)
  {
    clear();
    spacing = std::max(0.02, spacing);
    const double length = distance(robot, leader);
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / spacing)));
    for (std::size_t index = 0; index <= steps; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      const Point2 point{
        robot.x + ratio * (leader.x - robot.x),
        robot.y + ratio * (leader.y - robot.y)};
      append(point, 0.0);
    }
  }

  bool append(const Point2 & point, double minimum_spacing)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    if (points_.empty()) {
      points_.push_back(TrailPoint{point, 0.0});
      return true;
    }
    const double segment = distance(points_.back().position, point);
    if (segment < std::max(0.0, minimum_spacing)) {
      return false;
    }
    points_.push_back(TrailPoint{point, points_.back().station + segment});
    return true;
  }

  std::optional<Point2> sample(double station) const
  {
    if (points_.empty()) {
      return std::nullopt;
    }
    station = clamp(station, points_.front().station, points_.back().station);
    const auto upper = std::lower_bound(
      points_.begin(), points_.end(), station,
      [](const TrailPoint & point, double value) {return point.station < value;});
    if (upper == points_.begin()) {
      return upper->position;
    }
    if (upper == points_.end()) {
      return points_.back().position;
    }
    const auto lower = std::prev(upper);
    const double span = upper->station - lower->station;
    const double ratio = span > 1e-9 ? (station - lower->station) / span : 0.0;
    return Point2{
      lower->position.x + ratio * (upper->position.x - lower->position.x),
      lower->position.y + ratio * (upper->position.y - lower->position.y)};
  }

  double nearest_station(const Point2 & query) const
  {
    return nearest_station(
      query, -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
  }

  double nearest_station(
    const Point2 & query, double minimum_station, double maximum_station) const
  {
    if (points_.empty()) {
      return 0.0;
    }
    if (points_.size() == 1U) {
      return points_.front().station;
    }
    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_station = clamp(
      points_.front().station, minimum_station, maximum_station);
    for (std::size_t index = 1; index < points_.size(); ++index) {
      const auto & first = points_[index - 1U];
      const auto & second = points_[index];
      if (second.station < minimum_station || first.station > maximum_station) {
        continue;
      }
      const double dx = second.position.x - first.position.x;
      const double dy = second.position.y - first.position.y;
      const double length_squared = dx * dx + dy * dy;
      double projection = length_squared > 1e-12 ? clamp(
        ((query.x - first.position.x) * dx + (query.y - first.position.y) * dy) /
        length_squared, 0.0, 1.0) : 0.0;
      const double segment_station = second.station - first.station;
      if (segment_station > 1e-12) {
        projection = clamp(
          projection,
          clamp((minimum_station - first.station) / segment_station, 0.0, 1.0),
          clamp((maximum_station - first.station) / segment_station, 0.0, 1.0));
      }
      const double px = first.position.x + projection * dx;
      const double py = first.position.y + projection * dy;
      const double error_x = query.x - px;
      const double error_y = query.y - py;
      const double error_squared = error_x * error_x + error_y * error_y;
      if (error_squared < best_distance_squared) {
        best_distance_squared = error_squared;
        best_station = first.station + projection * (second.station - first.station);
      }
    }
    return best_station;
  }

private:
  std::vector<TrailPoint> points_;
};

struct TrailControlLimits
{
  double desired_distance{2.0};
  double gap_gain{0.9};
  double maximum_linear{1.0};
  double maximum_angular{1.2};
  double minimum_lookahead{0.45};
  double maximum_lookahead{1.20};
  double lookahead_time{0.8};
  // A 2 m-radius U-turn at a 1.0 m/s leader speed needs about 0.5 rad/s.
  // Keep enough linear-speed reserve to follow it within the 1.2 m/s limit.
  double curvature_gain{0.40};
  double minimum_curve_speed_scale{0.45};
  // Straight/turn mode hysteresis is based on a two-metre local trail chord.
  // A 0.10 1/m exit threshold corresponds to a ten-metre radius; the larger
  // entry threshold prevents noisy target centroids from repeatedly changing
  // modes on a straight.
  double curve_detection_distance{2.0};
  double curve_mode_enter_curvature{0.16};
  double curve_mode_exit_curvature{0.10};
  double straight_lookahead{1.50};
  double turn_minimum_lookahead{0.60};
  double turn_maximum_lookahead{0.80};
  double straight_heading_gain{1.20};
  double straight_cross_track_gain{0.90};
  double straight_heading_deadband{3.0 * M_PI / 180.0};
  double straight_cross_track_deadband{0.05};
  double straight_maximum_angular{0.35};
  double maximum_turn_linear{0.45};
  double rotate_to_heading_angle{0.65};
  double rotate_gain{1.8};
  double minimum_rotate_angular{0.35};
};

struct FollowCommand
{
  double linear{0.0};
  double angular{0.0};
  double gap{0.0};
  double curvature{0.0};
  double local_path_curvature{0.0};
  double heading_error{0.0};
  double cross_track_error{0.0};
  double robot_station{0.0};
  double goal_station{0.0};
  Point2 carrot;
  bool curve_mode{false};
  bool valid{false};
};

inline double three_point_curvature(
  const Point2 & first, const Point2 & middle, const Point2 & last)
{
  const double first_middle = distance(first, middle);
  const double middle_last = distance(middle, last);
  const double first_last = distance(first, last);
  const double denominator = first_middle * middle_last * first_last;
  if (denominator <= 1.0e-6) {
    return 0.0;
  }
  const double cross =
    (middle.x - first.x) * (last.y - first.y) -
    (middle.y - first.y) * (last.x - first.x);
  return 2.0 * cross / denominator;
}

inline FollowCommand trail_command(
  const LeaderTrail & trail, const Point2 & robot, double robot_yaw,
  double leader_speed, double current_linear, const TrailControlLimits & limits,
  double minimum_robot_station = -std::numeric_limits<double>::infinity(),
  double maximum_robot_station = std::numeric_limits<double>::infinity(),
  bool previous_curve_mode = false)
{
  FollowCommand command;
  if (trail.size() < 2U || !std::isfinite(robot_yaw) || !std::isfinite(leader_speed)) {
    return command;
  }
  command.robot_station = trail.nearest_station(
    robot, minimum_robot_station, maximum_robot_station);
  const double leader_station = trail.end_station();
  command.gap = std::max(0.0, leader_station - command.robot_station);
  command.goal_station = std::max(0.0, leader_station - limits.desired_distance);

  const double curve_end_station = std::min(
    leader_station, command.robot_station + limits.curve_detection_distance);
  const double curve_middle_station = 0.5 * (
    command.robot_station + curve_end_station);
  const auto curve_first = trail.sample(command.robot_station);
  const auto curve_middle = trail.sample(curve_middle_station);
  const auto curve_last = trail.sample(curve_end_station);
  if (curve_first && curve_middle && curve_last &&
    curve_end_station - command.robot_station >= 0.50)
  {
    command.local_path_curvature = three_point_curvature(
      *curve_first, *curve_middle, *curve_last);
  }
  command.curve_mode = previous_curve_mode ?
    std::abs(command.local_path_curvature) >= limits.curve_mode_exit_curvature :
    std::abs(command.local_path_curvature) >= limits.curve_mode_enter_curvature;

  const double turn_speed_ratio = limits.maximum_linear > 1.0e-6 ?
    clamp(std::abs(current_linear) / limits.maximum_linear, 0.0, 1.0) : 0.0;
  const double turn_lookahead =
    limits.turn_minimum_lookahead +
    turn_speed_ratio * (limits.turn_maximum_lookahead - limits.turn_minimum_lookahead);
  const double lookahead = command.curve_mode ? turn_lookahead : limits.straight_lookahead;
  const double carrot_station = std::min(command.goal_station, command.robot_station + lookahead);
  const auto carrot = trail.sample(carrot_station);
  if (!carrot) {
    return command;
  }
  command.carrot = *carrot;
  const double dx = carrot->x - robot.x;
  const double dy = carrot->y - robot.y;
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);
  const double body_x = cosine * dx + sine * dy;
  const double body_y = -sine * dx + cosine * dy;
  const double range_squared = std::max(1e-6, body_x * body_x + body_y * body_y);
  const double heading_error = std::atan2(body_y, body_x);
  command.curvature = 2.0 * body_y / range_squared;

  const double gap_error = command.gap - limits.desired_distance;
  double requested_linear = leader_speed + limits.gap_gain * gap_error;
  requested_linear = clamp(requested_linear, 0.0, limits.maximum_linear);
  if (command.curve_mode) {
    const double curve_scale = std::max(
      limits.minimum_curve_speed_scale,
      1.0 / (1.0 + limits.curvature_gain * std::abs(command.curvature)));
    requested_linear = std::min(
      requested_linear * curve_scale, limits.maximum_turn_linear);
  }

  if (leader_speed < 0.03 &&
    command.goal_station <= command.robot_station + 0.03 && gap_error <= 0.0)
  {
    requested_linear = 0.0;
  }
  if (body_x < 0.0 || std::abs(heading_error) >= limits.rotate_to_heading_angle) {
    command.linear = 0.0;
    command.heading_error = heading_error;
    double rotate = clamp(
      limits.rotate_gain * heading_error, -limits.maximum_angular, limits.maximum_angular);
    if (std::abs(rotate) < limits.minimum_rotate_angular && std::abs(heading_error) > 1.0e-6) {
      rotate = std::copysign(limits.minimum_rotate_angular, heading_error);
    }
    command.angular = rotate;
  } else if (command.curve_mode) {
    command.linear = requested_linear;
    command.angular = clamp(
      requested_linear * command.curvature,
      -limits.maximum_angular, limits.maximum_angular);
    command.heading_error = heading_error;
  } else {
    const double line_start_station = std::max(
      trail.points().front().station, command.robot_station - 0.25);
    const auto line_start = trail.sample(line_start_station);
    const auto line_end = trail.sample(carrot_station);
    if (!line_start || !line_end || distance(*line_start, *line_end) < 1.0e-4) {
      return FollowCommand{};
    }
    const double path_heading = std::atan2(
      line_end->y - line_start->y, line_end->x - line_start->x);
    command.heading_error = wrap_angle(path_heading - robot_yaw);
    const double path_sine = std::sin(path_heading);
    const double path_cosine = std::cos(path_heading);
    command.cross_track_error =
      -path_sine * (robot.x - line_start->x) +
      path_cosine * (robot.y - line_start->y);
    command.linear = requested_linear;
    if (std::abs(command.heading_error) <= limits.straight_heading_deadband &&
      std::abs(command.cross_track_error) <= limits.straight_cross_track_deadband)
    {
      command.angular = 0.0;
    } else {
      const double correction =
        limits.straight_heading_gain * command.heading_error -
        limits.straight_cross_track_gain * std::atan2(
        command.cross_track_error, std::max(0.25, limits.straight_lookahead));
      command.angular = clamp(
        correction, -std::min(limits.maximum_angular, limits.straight_maximum_angular),
        std::min(limits.maximum_angular, limits.straight_maximum_angular));
    }
  }
  command.valid = true;
  return command;
}

}  // namespace roka_terrain_mapping
