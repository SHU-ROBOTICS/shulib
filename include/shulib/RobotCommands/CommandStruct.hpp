// CommandStruct.hpp
#ifndef COMMAND_STRUCT_HPP
#define COMMAND_STRUCT_HPP

#include "CommandType.hpp"

struct CommandStruct {
    CommandType command;
    float x;       // x position
    float y;       // y position
    float heading; // Heading for MOVE_WITH_HEADING
    float speed;   // Speed for MOVE_WITH_HEADING
};

#endif // COMMAND_STRUCT_HPP
