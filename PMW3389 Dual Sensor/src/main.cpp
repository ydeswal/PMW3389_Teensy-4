#include <SPI.h>
#include <pgmspace.h>
#include <math.h>
#include "PMW3389.h"

#define SS1  9
#define SS2  10

// Earth radius in meters for great circle calculations
#define EARTH_RADIUS_M 6371000.0

// Spherical treadmill parameters (from research paper)
#define TREADMILL_RADIUS 0.10  // 10cm radius from paper
#define SENSOR_SEPARATION 0.02  // Distance between sensors in meters

// Scale factor to convert sensor counts to approximate movement
// Adjust this based on your sensor's CPI and calibration
#define COUNTS_TO_METERS 0.001

// Communication constants (similar to sample.cpp)
const String delimiter = ",";
const int decimalPlaces = 6;
bool headerSent = false;
PMW3389 sensor1, sensor2;
int xydat_1[2] = {0, 0};
int xydat_2[2] = {0, 0};
unsigned long currTime;
unsigned long pollTimer = 0;
unsigned long startTime = 0;  // Track acquisition start time

// Position tracking variables for spherical treadmill (corrected approach)
double current_theta = 0.0;      // Current angle in radians (spherical coordinates)
double current_phi = 0.0;        // Current azimuth in radians (spherical coordinates) 
double total_distance = 0.0;     // Total arc length traveled on sphere surface

// Navigation variables for virtual map integration
double current_x = 0.0;          // X position in virtual space (meters)
double current_y = 0.0;          // Y position in virtual space (meters)
double current_heading = 0.0;    // Current heading/bearing in degrees (0-360)
double current_speed = 0.0;      // Current speed in m/s
double angular_velocity = 0.0;   // Angular velocity in degrees/s

// Movement tracking
double prev_time = 0.0;          // Previous timestamp for speed calculation
double velocity_x = 0.0;         // X velocity component  
double velocity_y = 0.0;         // Y velocity component

// Sensor configuration (spherical coordinates from paper)
// Sensor 1 at (N2°, E0°) = (theta=88°, phi=0°) 
// Sensor 2 at (N23°, E57°) = (theta=67°, phi=57°)
const double sensor1_theta = deg_to_rad(88.0);  // 2° from north pole
const double sensor1_phi = deg_to_rad(0.0);     // 0° longitude
const double sensor2_theta = deg_to_rad(67.0);  // 23° from north pole  
const double sensor2_phi = deg_to_rad(57.0);    // 57° longitude

void setup() {
  Serial.begin(115200);  // Match sample.cpp baud rate
  Serial.println("Starting dual PMW3389 sensor initialization...");
  sensor1.begin(SS1);
  sensor2.begin(SS2);
  delay(250);
  if (sensor1.begin(SS1))
    Serial.println("Sensor1 initialization successed");
  else
    Serial.println("Sensor1 initialization failed");
  if (sensor2.begin(SS2))
    Serial.println("Sensor2 initialization successed");
  else
    Serial.println("Sensor2 initialization failed");
  sensor1.setCPI(1600);
  sensor2.setCPI(1600);
  int cpi1 = sensor1.getCPI();
  int cpi2 = sensor2.getCPI();
  Serial.print("Sensor 1 CPI: ");
  Serial.println(cpi1);
  Serial.print("Sensor 2 CPI: ");
  Serial.println(cpi2);
  if (cpi1 != cpi2 || cpi1 != 1600)
    Serial.println("WARNING: CPI initialization failed");
  else
    Serial.println("Both sensors detected successfully! Starting polling mode...");
  sensor1.readBurst();
  sensor2.readBurst();
  
  // Initialize start time for timestamp calculation
  startTime = millis();
}

// Convert degrees to radians
double deg_to_rad(double degrees) {
  return degrees * (M_PI / 180.0);
}

// Convert radians to degrees
double rad_to_deg(double radians) {
  return radians * (180.0 / M_PI);
}

// Vector operations for great circle method
struct Vector3D {
  double x, y, z;
  Vector3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

// Cross product of two 3D vectors
Vector3D cross_product(const Vector3D& a, const Vector3D& b) {
  return Vector3D(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

// Dot product of two 3D vectors
double dot_product(const Vector3D& a, const Vector3D& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Magnitude of a 3D vector
double magnitude(const Vector3D& v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Normalize a 3D vector
Vector3D normalize(const Vector3D& v) {
  double mag = magnitude(v);
  if (mag > 0) {
    return Vector3D(v.x / mag, v.y / mag, v.z / mag);
  }
  return Vector3D(0, 0, 0);
}

// Convert spherical coordinates to Cartesian (for sensor positions)
Vector3D spherical_to_cartesian(double theta, double phi, double radius = TREADMILL_RADIUS) {
  return Vector3D(
    radius * sin(theta) * cos(phi),
    radius * sin(theta) * sin(phi), 
    radius * cos(theta)
  );
}

// Great Circle Method for calculating spherical treadmill rotation (from paper)
// This implements the algorithm described in the research paper
Vector3D calculate_rotation_axis(const Vector3D& motion1, const Vector3D& motion2) {
  // Get sensor position vectors
  Vector3D M1 = spherical_to_cartesian(sensor1_theta, sensor1_phi);
  Vector3D M2 = spherical_to_cartesian(sensor2_theta, sensor2_phi);
  
  // Normalize motion vectors to get tangent vectors (gain-independent)
  Vector3D T1 = normalize(motion1);
  Vector3D T2 = normalize(motion2);
  
  // Step 1: Calculate P vectors (poles of great circles containing motion tangents)
  Vector3D cross1 = cross_product(M1, T1);
  Vector3D cross2 = cross_product(M2, T2);
  
  Vector3D P1 = normalize(cross1);
  Vector3D P2 = normalize(cross2);
  
  // Scale by treadmill radius
  P1.x *= TREADMILL_RADIUS; P1.y *= TREADMILL_RADIUS; P1.z *= TREADMILL_RADIUS;
  P2.x *= TREADMILL_RADIUS; P2.y *= TREADMILL_RADIUS; P2.z *= TREADMILL_RADIUS;
  
  // Step 2: Calculate Q vectors (poles of great circles containing rotation axis)
  Vector3D cross3 = cross_product(M1, P1);
  Vector3D cross4 = cross_product(M2, P2);
  
  Vector3D Q1 = normalize(cross3);
  Vector3D Q2 = normalize(cross4);
  
  // Scale by treadmill radius
  Q1.x *= TREADMILL_RADIUS; Q1.y *= TREADMILL_RADIUS; Q1.z *= TREADMILL_RADIUS;
  Q2.x *= TREADMILL_RADIUS; Q2.y *= TREADMILL_RADIUS; Q2.z *= TREADMILL_RADIUS;
  
  // Step 3: Calculate rotation axis A from cross product of Q vectors
  Vector3D cross5 = cross_product(Q1, Q2);
  Vector3D A = normalize(cross5);
  
  // Scale by treadmill radius
  A.x *= TREADMILL_RADIUS; A.y *= TREADMILL_RADIUS; A.z *= TREADMILL_RADIUS;
  
  return A;
}

// Calculate angular velocity magnitude from rotation axis and motion vector
double calculate_angular_velocity(const Vector3D& motion, const Vector3D& axis, const Vector3D& sensor_pos) {
  double motion_magnitude = magnitude(motion) * COUNTS_TO_METERS;
  double axis_dot_sensor = dot_product(axis, sensor_pos);
  double cos_angle = axis_dot_sensor / (magnitude(axis) * magnitude(sensor_pos));
  
  // Clamp cos_angle to valid range to avoid numerical errors
  cos_angle = fmax(-1.0, fmin(1.0, cos_angle));
  
  double sin_angle = sqrt(1.0 - cos_angle * cos_angle);
  
  if (sin_angle > 1e-6) {  // Avoid division by zero
    return motion_magnitude / sin_angle;
  }
  return 0.0;
}

// Update position based on spherical treadmill motion (corrected approach)
void update_spherical_position_and_distance(int s1_dx, int s1_dy, int s2_dx, int s2_dy, double delta_time) {
  // Convert sensor movements to 3D motion vectors
  Vector3D motion1(s1_dx * COUNTS_TO_METERS, s1_dy * COUNTS_TO_METERS, 0);
  Vector3D motion2(s2_dx * COUNTS_TO_METERS, s2_dy * COUNTS_TO_METERS, 0);
  
  // Calculate rotation axis using great circle method
  Vector3D rotation_axis = calculate_rotation_axis(motion1, motion2);
  
  // Get sensor positions
  Vector3D M1 = spherical_to_cartesian(sensor1_theta, sensor1_phi);
  
  // Calculate angular velocity
  double omega = calculate_angular_velocity(motion1, rotation_axis, M1);
  
  // Calculate arc length traveled on sphere surface
  double angular_displacement = omega * delta_time;
  double arc_length = angular_displacement * TREADMILL_RADIUS;
  
  // Accumulate total distance
  total_distance += fabs(arc_length);
  
  // Update spherical coordinates (simplified approach for virtual map)
  current_theta += angular_displacement * 0.1;  // Scale factor for mapping
  current_phi += angular_displacement * 0.1;
  
  // Normalize angles
  while (current_theta < 0) current_theta += 2 * M_PI;
  while (current_theta >= 2 * M_PI) current_theta -= 2 * M_PI;
  while (current_phi < 0) current_phi += 2 * M_PI; 
  while (current_phi >= 2 * M_PI) current_phi -= 2 * M_PI;
}

// Calculate comprehensive navigation data for virtual map (corrected approach)
void update_navigation_data(int s1_dx, int s1_dy, int s2_dx, int s2_dy, double delta_time) {
  // Calculate average movement for translation
  double avg_dx = (s1_dx + s2_dx) / 2.0;
  double avg_dy = (s1_dy + s2_dy) / 2.0;
  
  // Convert sensor counts to real-world movement
  double movement_x = avg_dx * COUNTS_TO_METERS;
  double movement_y = avg_dy * COUNTS_TO_METERS;
  
  // Update virtual position
  current_x += movement_x;
  current_y += movement_y;
  
  // Calculate velocities
  if (delta_time > 0) {
    velocity_x = movement_x / delta_time;
    velocity_y = movement_y / delta_time;
    current_speed = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  }
  
  // Calculate rotation using great circle method (corrected approach)
  Vector3D motion1(s1_dx * COUNTS_TO_METERS, s1_dy * COUNTS_TO_METERS, 0);
  Vector3D motion2(s2_dx * COUNTS_TO_METERS, s2_dy * COUNTS_TO_METERS, 0);
  
  // Calculate rotation axis
  Vector3D rotation_axis = calculate_rotation_axis(motion1, motion2);
  
  // Calculate angular displacement from rotation axis
  Vector3D M1 = spherical_to_cartesian(sensor1_theta, sensor1_phi);
  double omega = calculate_angular_velocity(motion1, rotation_axis, M1);
  double angular_displacement_rad = omega * delta_time;
  double angular_displacement_deg = rad_to_deg(angular_displacement_rad);
  
  // Update heading
  current_heading += angular_displacement_deg;
  
  // Normalize heading to 0-360 degrees
  while (current_heading < 0) current_heading += 360.0;
  while (current_heading >= 360) current_heading -= 360.0;
  
  // Calculate angular velocity in degrees/second
  if (delta_time > 0) {
    angular_velocity = angular_displacement_deg / delta_time;
  }
  
  // Update spherical position and distance using corrected method
  update_spherical_position_and_distance(s1_dx, s1_dy, s2_dx, s2_dy, delta_time);
}

// Send header function similar to sample.cpp
void sendHeader() {
  Serial.print(String(
      String("timestamp [ms]") + delimiter + "s1_dx [counts]" + delimiter + 
      "s1_dy [counts]" + delimiter + "s1_dt [s]" + delimiter + 
      "s2_dx [counts]" + delimiter + "s2_dy [counts]" + delimiter + 
      "s2_dt [s]" + delimiter + "x_pos [m]" + delimiter + 
      "y_pos [m]" + delimiter + "heading [deg]" + delimiter + 
      "water" + delimiter + "direction [deg]" + delimiter + 
      "frameCount" + delimiter + "distance [m]" + delimiter + 
      "speed [m/s]" + delimiter + "angular_vel [deg/s]" + delimiter + 
      "theta [deg]" + delimiter + "phi [deg]" + "\n"));
}

// Send data function similar to sample.cpp structure
void sendData(unsigned long timestamp, int s1_dx, int s1_dy, double dt, 
              int s2_dx, int s2_dy, unsigned long frameCount) {
  
  // Calculate milliseconds since acquisition start (like sample.cpp)
  unsigned long millisSinceStart = timestamp - startTime;
  
  // Convert sensor data to strings with specified decimal places
  const String timestampStr = String(millisSinceStart);
  const String s1_dxStr = String(s1_dx);
  const String s1_dyStr = String(s1_dy);
  const String dtStr = String(dt, decimalPlaces);
  const String s2_dxStr = String(s2_dx);
  const String s2_dyStr = String(s2_dy);
  const String xPosStr = String(current_x, decimalPlaces);
  const String yPosStr = String(current_y, decimalPlaces);
  const String headingStr = String(current_heading, 2);
  const String waterStr = String("0");  // Placeholder for water system
  const String directionStr = String(current_heading, 2);
  const String frameCountStr = String(frameCount);
  const String distanceStr = String(total_distance, decimalPlaces);
  const String speedStr = String(current_speed, decimalPlaces);
  const String angularVelStr = String(angular_velocity, decimalPlaces);
  const String thetaStr = String(rad_to_deg(current_theta), 8);  // Spherical coordinate theta
  const String phiStr = String(rad_to_deg(current_phi), 8);     // Spherical coordinate phi
  
  // Send CSV data (similar to sample.cpp structure)
  Serial.print(timestampStr + delimiter + s1_dxStr + delimiter + s1_dyStr + delimiter +
               dtStr + delimiter + s2_dxStr + delimiter + s2_dyStr + delimiter + dtStr + delimiter +
               xPosStr + delimiter + yPosStr + delimiter + headingStr + delimiter + waterStr + delimiter +
               directionStr + delimiter + frameCountStr + delimiter + distanceStr + delimiter +
               speedStr + delimiter + angularVelStr + delimiter + thetaStr + delimiter + phiStr + "\n");
}



void loop() {
  currTime = millis();
  
  // Send header once at start (similar to sample.cpp)
  if (!headerSent) {
    sendHeader();
    headerSent = true;
  }
  
  if(currTime > pollTimer) {
    PMW3389_DATA data1 = sensor1.readBurst();
    PMW3389_DATA data2 = sensor2.readBurst();
    
    // Store individual sensor readings
    xydat_1[0] = data1.dx;
    xydat_1[1] = data1.dy;
    xydat_2[0] = data2.dx;
    xydat_2[1] = data2.dy;
    
    // Calculate time delta for proper speed/velocity calculations
    double current_time = currTime / 1000.0;  // Convert to seconds
    double delta_time = (prev_time > 0) ? (current_time - prev_time) : 0.1;
    prev_time = current_time;
    
    // Update comprehensive navigation data
    update_navigation_data(xydat_1[0], xydat_1[1], xydat_2[0], xydat_2[1], delta_time);
    
    // Calculate frame count
    static unsigned long frameCount = 0;
    frameCount++;
    
    // Send data using sample.cpp-style communication
    sendData(currTime, xydat_1[0], xydat_1[1], delta_time, 
             xydat_2[0], xydat_2[1], frameCount);
    
    pollTimer = currTime + 100;  // 10Hz update rate
  }
}