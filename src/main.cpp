#include "main.h"
#include "pros/adi.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
#include "shulib/config.hpp"
#include <memory>

namespace {
    // Global variables in anonymous namespace
    struct GlobalVars {
        pros::Controller master{pros::E_CONTROLLER_MASTER};
        std::vector<pros::Motor> fl_motors;
        std::vector<pros::Motor> fr_motors;
        std::vector<pros::Motor> bl_motors;
        std::vector<pros::Motor> br_motors;
        pros::Rotation left{-20};
        pros::Rotation right{11};
        pros::Rotation back{7};
        shulib::OdomUnit leftOdom{nullptr, 2.75, -5.875};
        shulib::OdomUnit rightOdom{nullptr, 2.75, 5.875};
        shulib::OdomUnit backOdom{nullptr, 2.75, 4};
        std::unique_ptr<shulib::XDrive> drivetrain;
        std::unique_ptr<shulib::Chassis> chassis;
        pros::adi::Pneumatics doinker{'H', false};
        pros::adi::Pneumatics grabber{'G', false};
        pros::Motor intake{2};
        pros::Motor lift{9};
        pros::MotorGroup conveyor{{1, -10}};
        pros::MotorGroup wallStake{{3, -8}, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees};
        double conveyorSpeed{0.8};
        double intakeSpeed{1.0};
    };

    GlobalVars g;

    std::vector<int> parsePortString(const std::string& str) {
        std::vector<int> ports;
        size_t pos = 0;
        std::string token;
        std::string s = str;
        while ((pos = s.find(',')) != std::string::npos) {
            token = s.substr(0, pos);
            ports.push_back(std::stoi(token));
            s.erase(0, pos + 1);
        }
        if (!s.empty()) {
            ports.push_back(std::stoi(s));
        }
        return ports;
    }

    void loadConfig() {
        auto& config = shulib::Config::getInstance();
        if (!config.loadFromFile("robot_config.txt")) {
            // Set default values if config file doesn't exist
            config.set("frontLeft_ports", "-18,-19");
            config.set("frontRight_ports", "12,13");
            config.set("backLeft_ports", "-16,-17");
            config.set("backRight_ports", "15,14");
            config.set("track_width", 2.25);
            config.set("wheel_diameter", 2.0);
            config.set("lateral_kP", 10.0);
            config.set("lateral_kI", 0.0);
            config.set("lateral_kD", 3.0);
            config.set("angular_kP", 2.0);
            config.set("angular_kI", 0.0);
            config.set("angular_kD", 1.0);
            
            // Save default config
            config.saveToFile("robot_config.txt");
        }
    }

    void fifteen() {
        // right: pneumatics
        if (g.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            g.grabber.toggle();
        }
        // y : pneumatics #2
        if (g.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            g.doinker.toggle();
        }
        // r1 : wall stake setup
        if (g.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (abs(g.wallStake.get_position()) < 1) {
                g.wallStake.move_absolute(37, 50);
            } else {
                g.wallStake.move_absolute(0, 20);
            }
        }
        // r2 : wall stake lift
        if (g.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (abs(g.wallStake.get_position() - 37) < 2) {
                g.wallStake.move_absolute(140, 30);
            } else {
                g.wallStake.move_absolute(37, 30);
            }
        }
        // l2: intake, l1: outtake
        if (g.master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            g.intake.move(127 * g.intakeSpeed);
            g.conveyor.move(-127 * g.conveyorSpeed);
        } else if (g.master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            g.intake.move(-127 * g.intakeSpeed);
            g.conveyor.move(127 * g.conveyorSpeed);
        } else if (g.master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            g.conveyor.move(127 * g.conveyorSpeed);
        } else if (g.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            g.conveyor.move(-127 * g.conveyorSpeed);
        } else {
            g.intake.move(0);
            g.conveyor.move(0);
        }
    }
} // namespace

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Loading config...");

    loadConfig();
    auto& config = shulib::Config::getInstance();

    // Initialize motor groups from config
    std::vector<int> fl_ports = parsePortString(config.get<std::string>("frontLeft_ports"));
    std::vector<int> fr_ports = parsePortString(config.get<std::string>("frontRight_ports"));
    std::vector<int> bl_ports = parsePortString(config.get<std::string>("backLeft_ports"));
    std::vector<int> br_ports = parsePortString(config.get<std::string>("backRight_ports"));

    // Create motor vectors
    g.fl_motors.clear();
    g.fr_motors.clear();
    g.bl_motors.clear();
    g.br_motors.clear();

    for (int port : fl_ports) g.fl_motors.emplace_back(port);
    for (int port : fr_ports) g.fr_motors.emplace_back(port);
    for (int port : bl_ports) g.bl_motors.emplace_back(port);
    for (int port : br_ports) g.br_motors.emplace_back(port);

    // Create drivetrain with config values
    std::vector<int> fl_ports_array(fl_ports.begin(), fl_ports.end());
    std::vector<int> fr_ports_array(fr_ports.begin(), fr_ports.end());
    std::vector<int> bl_ports_array(bl_ports.begin(), bl_ports.end());
    std::vector<int> br_ports_array(br_ports.begin(), br_ports.end());

    g.drivetrain = std::make_unique<shulib::XDrive>(
        pros::MotorGroup(fl_ports_array.data(), fl_ports_array.size()),
        pros::MotorGroup(fr_ports_array.data(), fr_ports_array.size()),
        pros::MotorGroup(bl_ports_array.data(), bl_ports_array.size()),
        pros::MotorGroup(br_ports_array.data(), br_ports_array.size()),
        config.get<double>("track_width"),
        200, // RPM is typically fixed for a given motor
        config.get<double>("wheel_diameter")
    );

    // Create sensors
    shulib::OdomSensors sensors(&g.leftOdom, &g.rightOdom, &g.backOdom, nullptr);

    // Create PID controllers with config values
    shulib::ControllerSettings lateralSettings(
        config.get<double>("lateral_kP"),
        config.get<double>("lateral_kI"),
        config.get<double>("lateral_kD"),
        3, 1, 100, 3, 500, 20
    );

    shulib::ControllerSettings angularSettings(
        config.get<double>("angular_kP"),
        config.get<double>("angular_kI"),
        config.get<double>("angular_kD"),
        1, 1, 100, 3, 500, 20
    );

    // Create chassis
    g.chassis = std::make_unique<shulib::Chassis>(*g.drivetrain, sensors, lateralSettings, angularSettings);

    g.chassis->calibrate();
    g.chassis->setPose({36, -60, 0});

    pros::lcd::set_text(0, "Robot initialized!");

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", g.chassis->getPose().x);
            pros::lcd::print(1, "Y: %f", g.chassis->getPose().y);
            pros::lcd::print(2, "Theta: %f", g.chassis->getPose().theta);
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    g.wallStake.move_absolute(0, 10);

    while (true) {
        g.chassis->drive(g.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
                      g.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                      g.master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        fifteen();
        printf("wallStake: %f\n", g.wallStake.get_position());

        pros::delay(20);
    }
}