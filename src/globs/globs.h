// This is the only header file you should include to use globs.
// To define new objects you must modify this file and glob_types.h

#ifndef GLOBS_INCLUDED_H
#define GLOBS_INCLUDED_H

// Includes
#include <stdint.h>
#include "glob_types.h"

// Unique IDs given to globs are defined in this enumeration.
// Add an ID name to the enum for each new object.
// The bottom member of the enum should always be NUM_GLOBS which
// keeps track of the total number of objects (max number = 255)
typedef uint8_t glob_id_t;
enum
{
    GLO_ID_ASSERT_MESSAGE,      // Messages used by UI at top to keep ID's fairly constant
    GLO_ID_DEBUG_MESSAGE,
    GLO_ID_CAPTURE_DATA,
    GLO_ID_DRIVING_COMMAND,
    GLO_ID_CAPTURE_COMMAND,
    GLO_ID_STATUS_DATA,         // End UI message types
    GLO_ID_MOTION_COMMANDS,
    GLO_ID_RAW_ACCELS,
    GLO_ID_RAW_ANALOG,
    GLO_ID_GYROS_ACCELS,
    GLO_ID_ROLL_PITCH_YAW,
    GLO_ID_QUATERNION,
    GLO_ID_THETA_ZERO,
    GLO_ID_ODOMETRY,
    GLO_ID_MODES,
    GLO_ID_CURRENT_COMMANDS,

    NUM_GLOBS,
};

typedef struct glob_meta
{
    glob_id_t id;
    uint8_t num_bytes;
    uint16_t instance;
    void * ptr;

    glob_meta(void) :
        id(0), num_bytes(0), instance(1), ptr(0) {}

    glob_meta(glob_id_t id, uint8_t num_bytes, uint16_t instance, void * ptr) :
        id(id), num_bytes(num_bytes), instance(instance), ptr(ptr) {}

    template<class T>
    glob_meta(glob_id_t id, T const & data, uint16_t instance=1) :
        id(id), num_bytes(sizeof(data)), instance(instance), ptr((void *)&data) {}

} glob_meta_t;

#endif // GLOBS_INCLUDED_H
