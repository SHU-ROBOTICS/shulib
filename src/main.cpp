#include "main.h"
#include "pros/motor_group.hpp"

// Main initialization function
void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

class XDrive {
private:
  const int printInterval = 5;
  int printCounter = 0;

public:
  MotorGroup frontRight;
  MotorGroup backRight;
  MotorGroup frontLeft;
  MotorGroup backLeft;
  pros::IMU *imu;

  XDrive(const std::vector<std::int8_t> &frontRightPorts,
         const std::vector<std::int8_t> &backRightPorts,
         const std::vector<std::int8_t> &frontLeftPorts,
         const std::vector<std::int8_t> &backLeftPorts,
         pros::IMU *imu = nullptr)
      : frontRight(frontRightPorts), backRight(backRightPorts),
        frontLeft(frontLeftPorts), backLeft(backLeftPorts), imu(imu) {}

  void move(int x, int y, int r) {
    // Field-centric control using IMU
    if (imu) {
      double angle = imu->get_rotation() * M_PI / 180.0;
      double cosA = cos(angle);
      double sinA = sin(angle);
      int rotatedX = x * cosA - y * sinA;
      int rotatedY = x * sinA + y * cosA;
      x = rotatedX;
      y = rotatedY;
    }

    // Calculate motor powers for X-drive
    frontLeft.move((y + x + r) * -1);
    backLeft.move((y - x + r) * -1);
    frontRight.move(y - x - r);
    backRight.move(y + x - r);
  }

  void send() {
    if (printCounter == printInterval) {
      printf("POS:%d,%d ROT:%d\n", 0, 0,
             imu ? (int(imu->get_rotation()) % 360 + 360) % 360 : 0);
      printCounter = 0;
    } else {
      printCounter++;
    }
  }
};

class OdometryUnit {
public:
  Rotation encoder;
  int32_t lastValue;
  double wheelRadius;
  double distance;

  OdometryUnit(const std::int8_t &port, const double &radius)
      : encoder(port), wheelRadius(radius) {
    lastValue = encoder.get_angle();
  }

  double get_travel() {
    int32_t currentValue = encoder.get_position();
    int32_t deltaValue = currentValue - lastValue;
    lastValue = currentValue;

    // Convert degrees to radians & calculate travel distance
    double deltaAngleRadians = deltaValue * (M_PI / 180.0);
    double travel = wheelRadius * deltaAngleRadians;
    return travel;
  }

  double get_total_travel() {
    double totalAngleRadians = encoder.get_position() * (M_PI / 180.0);
    return wheelRadius * totalAngleRadians;
  }
};

class OdometryController {
public:
  OdometryUnit &left;
  OdometryUnit &right;
  OdometryUnit &back;
  double theta_r; // Orientation at last reset (radians)
  double theta_0; // Previous orientation (radians)
  double x;
  double y;

  // distance from the "tracking center" to each wheel (inches)
  double sL, sR, sB;

  OdometryController(OdometryUnit &leftUnit, OdometryUnit &rightUnit,
                     OdometryUnit &backUnit, double initialOrientation = 0.0,
                     double initialX = 0.0, double initialY = 0.0)
      : left(leftUnit), right(rightUnit), back(backUnit),
        theta_r(initialOrientation), theta_0(initialOrientation), x(initialX),
        y(initialY), sL(leftUnit.distance), sR(rightUnit.distance),
        sB(backUnit.distance) {}

  void reset(double newOrientation = 0.0, double newX = 0.0,
             double newY = 0.0) {
    theta_r = newOrientation;
    theta_0 = newOrientation;
    x = newX;
    y = newY;

    left.lastValue = left.encoder.get_position();
    right.lastValue = right.encoder.get_position();
    back.lastValue = back.encoder.get_position();
  }

  // void cycle() {
  //   double deltaL = left.get_total_travel();
  //   double deltaR = right.get_total_travel();
  //   double deltaB = back.get_total_travel();

  //   double deltaTheta = (deltaL - deltaR) / (sL + sR);
  //   double absoluteTheta = prevOrientation + deltaTheta;

  //   double deltaX_local = deltaB - (deltaTheta * sB);
  //   double deltaY_local = (deltaL + deltaR) / 2.0;

  //   double avgTheta = prevOrientation + (deltaTheta / 2.0);
  //   double sinTheta = sin(avgTheta);
  //   double cosTheta = cos(avgTheta);

  //   double deltaX_global =
  //       (deltaX_local * cosTheta) - (deltaY_local * sinTheta);
  //   double deltaY_global =
  //       (deltaX_local * sinTheta) + (deltaY_local * cosTheta);

  //   position[0] += deltaX_global;
  //   position[1] += deltaY_global;

  //   prevOrientation = absoluteTheta;
  // }


  // This function should be called periodically to update the robot's position
  void cycle() {
    // Step 2: Get the travel distances since the last cycle
    double deltaL = left.get_travel();
    double deltaR = right.get_travel();
    double deltaS = back.get_travel();

    // Step 4: Get total distances since last reset
    double totalL = left.get_total_travel();
    double totalR = right.get_total_travel();

    // ΔLr and ΔRr (since last reset)
    double deltaLr = totalL;
    double deltaRr = totalR;

    // Step 5: Calculate new absolute orientation
    double theta_new = theta_r + (deltaLr - deltaRr) / (sL + sR);

    // Step 6: Calculate change in orientation
    double deltaTheta = theta_new - theta_0;

    // Step 7 & 8: Calculate the local offset
    double deltaX_local = 0.0;
    double deltaY_local = 0.0;

    if (deltaTheta == 0.0) {
      // If the robot didn't rotate, it's a straight movement
      deltaX_local = deltaS;
      deltaY_local = (deltaL + deltaR) / 2.0;
    } else {
      // Calculate the radius of the movement for each tracking wheel
      double radiusL = deltaL / deltaTheta;
      double radiusR = deltaR / deltaTheta;
      double radiusS = deltaS / deltaTheta;

      // Calculate the local movement components
      double offsetX = 2 * sin(deltaTheta / 2.0) * (radiusS + sB);
      double offsetY = 2 * sin(deltaTheta / 2.0) * (radiusR + sR);

      deltaX_local = offsetX;
      deltaY_local = offsetY;
    }

    // Step 9: Calculate the average orientation
    double theta_middle = theta_0 + deltaTheta / 2.0;

    // Step 10: Rotate the local offset to the global coordinate system
    double cosTheta = cos(theta_middle);
    double sinTheta = sin(theta_middle);

    double deltaX_global = deltaX_local * cosTheta - deltaY_local * sinTheta;
    double deltaY_global = deltaX_local * sinTheta + deltaY_local * cosTheta;

    // Step 11: Update the global position
    x += deltaX_global;
    y += deltaY_global;

    // Update previous orientation
    theta_0 = theta_new;

    // Optional: Print the position for debugging
    printf("Position: (%.2f, %.2f), Orientation: %.2f degrees\n",
          x, y,
          theta_new * 180.0 / M_PI);
  }
};

void opcontrol() {
  // sample for X drive
  Controller master(CONTROLLER_MASTER);
  IMU imu(10);
  XDrive drive({19, 20}, {11, 12}, {17, 18}, {13, 14}, &imu);

  while (true) {
    // drive.move(master.get_analog(ANALOG_LEFT_X),
    //            master.get_analog(ANALOG_LEFT_Y),
    //            master.get_analog(ANALOG_RIGHT_X));

    // drive.send();

    pros::delay(20);
  }
}