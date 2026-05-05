
//! For flashing use:

// make remote
// make v1_motor
// make v2_motor

// These translate to:
// idf.py -DDEVICE=remote build flash monitor
// idf.py -DDEVICE=v1 build flash monitor
// idf.py -DDEVICE=v2 build flash monitor
#include "sdkconfig.h"

#ifdef CONFIG_DEVICE_REMOTE_MOTOR_CONTROLLER
#include "remote/remote.h"
#endif

#ifdef CONFIG_DEVICE_V1_MOTOR_CONTROLLER
#include "V1_Motor/v1_motor.h"
#endif

#ifdef CONFIG_DEVICE_V2_MOTOR_CONTROLLER
#include "V2_Motor/v2_motor.h"
#endif

extern "C" void app_main(void)
{
#ifdef CONFIG_DEVICE_REMOTE_MOTOR_CONTROLLER
    remote_init();
#elif defined(CONFIG_DEVICE_V1_MOTOR_CONTROLLER)
    v1_motor_init();
#elif defined(CONFIG_DEVICE_V2_MOTOR_CONTROLLER)
    v2_motor_init();
#endif
}