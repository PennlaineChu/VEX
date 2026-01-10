class Odom
{
private:
  float ForwardTracker_center_distance;
  float SidewaysTracker_center_distance;
  float ForwardTracker_position;
  float SideWaysTracker_position;
  float LeftTracker_center_distance;
  float RightTracker_center_distance;
  float BackTracker_center_distance;
  float LeftTracker_position;
  float RightTracker_position;
  float BackTracker_position;
public:
  float X_position;
  float Y_position;
  float orientation_deg;
  void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
  void update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg);
  void set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance);
  void set_three_wheel_distances(float LeftTracker_center_distance, float RightTracker_center_distance, float BackTracker_center_distance);
  void set_three_wheel_position(float X_position, float Y_position, float orientation_deg, float LeftTracker_position, float RightTracker_position, float BackTracker_position);
  void update_three_wheel_position(float LeftTracker_position, float RightTracker_position, float BackTracker_position, float orientation_deg);
};