#ifndef MOVE_WITH_HEADING_COMMAND_HPP
#define MOVE_WITH_HEADING_COMMAND_HPP

#include "Command.hpp"
#include <iostream>

class MoveWithHeadingCommand : public Command {
private:
    float x, y, heading, speed;

public:
    MoveWithHeadingCommand(float xPos, float yPos, float head, float spd)
        : x(xPos), y(yPos), heading(head), speed(spd) {}

    void execute() {
        // Logic to move the robot to (x, y) while keeping the heading and speed
        moveTo(x, y, heading, speed);
    }

private:
    void moveTo(float x, float y, float heading, float speed) {
        // Implementation for moving the robot
        // Example: control motors, set heading, adjust speed, etc.
    }
};

#endif // MOVE_WITH_HEADING_COMMAND_HPP
