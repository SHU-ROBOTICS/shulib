#ifndef AUTONOMOUS_COMMANDS_H
#define AUTONOMOUS_COMMANDS_H

#include <cstddef> 

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_MOVE_WITH_HEADING,
    CMD_PICK_UP,
    CMD_PLACE,
    CMD_SCOOP,
    CMD_RELEASE,
    CMD_CLASP
} CommandType;

typedef struct {
    CommandType command;
    float x;
    float y;
    float heading;
    float speed;
} Command;

extern Command autonomous_commands[];
extern const size_t num_autonomous_commands;

#ifdef __cplusplus
}
#endif

#endif 
