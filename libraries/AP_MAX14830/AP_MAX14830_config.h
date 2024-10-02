#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/msp.h>

#ifndef AP_MAX14830_ENABLED
 #define AP_MAX14830_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS)
#endif

/////////////////////////////////////////////////////////////////////////////
//  Custom Hardware Pins for LZC
#ifndef HAL_GPIO_PIN_POS_LIGHTS
 #define HAL_GPIO_PIN_POS_LIGHTS 101
#endif

#ifndef HAL_GPIO_PIN_BCN_LIGHTS
 #define HAL_GPIO_PIN_BCN_LIGHTS 102
#endif

#ifndef HAL_GPIO_PIN_CHUTE_RELEASE
 #define HAL_GPIO_PIN_CHUTE_RELEASE 103
#endif

#ifndef HAL_GPIO_PIN_BLN_RELEASE
 #define HAL_GPIO_PIN_BLN_RELEASE 104
#endif

#ifndef HAL_GPIO_PIN_HSTM_POWER
 #define HAL_GPIO_PIN_HSTM_POWER 105
#endif

#ifndef HAL_GPIO_EXT_WDOG_STATUS
 #define HAL_GPIO_EXT_WDOG_STATUS 99
#endif

#ifndef HAL_GPIO_EXT_WDOG_RESET
 #define HAL_GPIO_EXT_WDOG_RESET 98
#endif

#ifndef HAL_GPIO_PIN_EXT_WDOG
 #define HAL_GPIO_PIN_EXT_WDOG 97
#endif

#ifndef HAL_GPIO_PIN_WING_LIMIT
 #define HAL_GPIO_PIN_WING_LIMIT 54
#endif

#ifndef HAL_ANALOG_PIN_SERV0
 #define HAL_ANALOG_PIN_SERV0 12
#endif

#ifndef HAL_ANALOG_PIN_BOARD_TEMP
 #define HAL_ANALOG_PIN_BOARD_TEMP 13
#endif
