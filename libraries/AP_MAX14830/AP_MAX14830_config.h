#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/msp.h>

#ifndef AP_MAX14830_ENABLED
 #define AP_MAX14830_ENABLED (CONFIG_HAL_BOARD != HAL_BOARD_SITL)
#endif

/////////////////////////////////////////////////////////////////////////////
//  Custom Hardware Pins for LZC
#ifndef HAL_GPIO_PIN_POS_LIGHTS
 #define HAL_GPIO_PIN_POS_LIGHTS -1
#endif

#ifndef HAL_GPIO_PIN_BCN_LIGHTS
 #define HAL_GPIO_PIN_BCN_LIGHTS -1
#endif

#ifndef HAL_GPIO_PIN_CHUTE_RELEASE
 #define HAL_GPIO_PIN_CHUTE_RELEASE -1
#endif

#ifndef HAL_GPIO_PIN_BLN_RELEASE
 #define HAL_GPIO_PIN_BLN_RELEASE -1
#endif

#ifndef HAL_GPIO_PIN_HSTM_POWER
 #define HAL_GPIO_PIN_HSTM_POWER -1
#endif

#ifndef HAL_GPIO_EXT_WDOG_STATUS
 #define HAL_GPIO_EXT_WDOG_STATUS -1
#endif

#ifndef HAL_GPIO_EXT_WDOG_RESET
 #define HAL_GPIO_EXT_WDOG_RESET -1
#endif

#ifndef HAL_GPIO_PIN_EXT_WDOG
 #define HAL_GPIO_PIN_EXT_WDOG -1
#endif

#ifndef HAL_GPIO_PIN_WING_LIMIT
 #define HAL_GPIO_PIN_WING_LIMIT -1
#endif

#ifndef HAL_ANALOG_PIN_SERV0
 #define HAL_ANALOG_PIN_SERV0 -1
#endif

#ifndef HAL_ANALOG_PIN_BOARD_TEMP
 #define HAL_ANALOG_PIN_BOARD_TEMP -1
#endif
