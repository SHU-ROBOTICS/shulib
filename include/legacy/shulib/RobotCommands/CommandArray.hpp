// CommandArray.hpp
#ifndef COMMAND_ARRAY_HPP
#define COMMAND_ARRAY_HPP

#include <cstddef> 
#include "CommandStruct.hpp"

// Declare the array of commands
extern CommandStruct autonomous_commands[];
extern size_t autonomous_commands_count;

#endif // COMMAND_ARRAY_HPP
