#ifndef GHOST_ROS__V5_PORT_CONFIG_HPP
#define GHOST_ROS__V5_PORT_CONFIG_HPP

#include <string>
#include <map>

namespace ghost_v5_config
{
    enum v5_motor_id_enum
    {
        DRIVE_LEFT_FRONT_MOTOR          = 18,
        DRIVE_LEFT_BACK_MOTOR           = 17,

        DRIVE_RIGHT_FRONT_MOTOR         = 14,
        DRIVE_RIGHT_BACK_MOTOR          = 13,

        DRIVE_BACK_LEFT_REAR_MOTOR      = 15,
        DRIVE_BACK_LEFT_FRONT_MOTOR     = 11,

        DRIVE_BACK_RIGHT_REAR_MOTOR     = 9,
        DRIVE_BACK_RIGHT_FRONT_MOTOR    = 10,

        TURRET_MOTOR                    = 1,
        INTAKE_MOTOR                    = 3,
        SHOOTER_RIGHT_MOTOR             = 6,
        SHOOTER_LEFT_MOTOR              = 8,
        INDEXER_MOTOR                   = 4
    };

    enum v5_sensor_id_enum
    {
        STEERING_LEFT_ENCODER           = 2,
        STEERING_RIGHT_ENCODER          = 5,
        STEERING_BACK_ENCODER           = 12,
        TURRET_ENCODER                  = 7
    };

    enum v5_pneumatic_id_enum
    {
        FLYWHEEL_TILT   = 1,
        INDEXER_ROOF    = 2,
        ENDGAME         = 3,
    };
    
} // ghost_v5_config

#endif // GHOST_ROS__V5_PORT_CONFIG_HPP
