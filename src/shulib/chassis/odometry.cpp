// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "pros/rtos.hpp"
#include "shulib/util.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/odomUnit.hpp"

// tracking thread
pros::Task* trackingTask = nullptr;

// telemetry thread
pros::Task* telemetryTask = nullptr;

// global variables
shulib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
shulib::Drivetrain drive(0, 0, 0); // the drivetrain to be used for odometry
shulib::Pose odomPose(0, 0, 0); // the pose of the robot
shulib::Pose odomSpeed(0, 0, 0); // the speed of the robot
shulib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

float prevVertical = 0;
float prevLeft = 0;
float prevRight = 0;
float prevHorizontal = 0;
float prevBack = 0;
float prevImu = 0;

int telemetryDelay = 200;

void shulib::setSensors(shulib::OdomSensors sensors, shulib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

shulib::Pose shulib::getPose(bool radians) {
    if (radians) return odomPose;
    else return shulib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void shulib::setPose(shulib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = shulib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

shulib::Pose shulib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return shulib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

shulib::Pose shulib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return shulib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

shulib::Pose shulib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void shulib::update() {
    // get the current sensor values
    float leftRaw = 0;
    float rightRaw = 0;
    float backRaw = 0;
    float imuRaw = 0;
    if (odomSensors.left != nullptr) leftRaw = odomSensors.left->get_travel();
    if (odomSensors.right != nullptr) rightRaw = odomSensors.right->get_travel();
    if (odomSensors.back != nullptr) backRaw = odomSensors.back->get_travel();
    if (odomSensors.imu != nullptr) imuRaw = degToRad(odomSensors.imu->get_rotation());

    // calculate the change in sensor values
    float deltaLeft = leftRaw - prevLeft;
    float deltaRight = rightRaw - prevRight;
    float deltaBack = backRaw - prevBack;
    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevLeft = leftRaw;
    prevRight = rightRaw;
    prevBack = backRaw;
    prevImu = imuRaw;

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    float heading = odomPose.theta;

    // calculate the heading using the left and right tracking wheels
    if (odomSensors.left != nullptr && odomSensors.right != nullptr)
        heading -= (deltaLeft - deltaRight) /
                   (odomSensors.left->get_offset() - odomSensors.right->get_offset());
    // else, if the inertial sensor exists, use it
    else if (odomSensors.imu != nullptr) heading += deltaImu;
    else heading += 0;

    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    shulib::OdomUnit* verticalWheel = nullptr;
    shulib::OdomUnit* horizontalWheel = nullptr;

    // visit later: why not use both left and right tracking wheels?
    if (odomSensors.left != nullptr) verticalWheel = odomSensors.left;
    else if (odomSensors.right != nullptr) verticalWheel = odomSensors.right;
    else verticalWheel = nullptr;


    if (odomSensors.back != nullptr) horizontalWheel = odomSensors.back;
    else horizontalWheel = nullptr;

    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->get_travel();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->get_travel();

    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->get_offset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->get_offset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    shulib::Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);
}

void shulib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
    if (telemetryTask == nullptr && telemetryDelay > 0) {
        telemetryTask = new pros::Task {[=] {
            while (true) {
                printf("POS:%d,%d ROT:%d\n", (int) odomPose.x, (int) odomPose.y, (int) odomPose.theta);
                pros::delay(telemetryDelay);
            }
        }};
    }
}