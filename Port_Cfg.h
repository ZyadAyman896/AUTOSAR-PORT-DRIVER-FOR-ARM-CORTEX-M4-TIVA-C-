/*
 * Port_Cfg.h
 *
 *  Created on: Aug 31, 2024
 *      Author: Zyad Ayman
 */

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT               (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API               (STD_ON)

/* Pre-compile option for presence of Port_SetPinDirection API */
#define PORT_SET_PIN_DIRECTION_API          (STD_ON)

/* Pre-compile option for presence of Port_SetPinMode API */
#define PORT_SET_PIN_MODE_API               (STD_ON)

/* Number of PINS */
#define PORT_CONFIGURED_PINS                (43U)

/* Tiva-c Ports */
#define PORTA (0U)
#define PORTB (1U)
#define PORTC (2U)
#define PORTD (3U)
#define PORTE (4U)
#define PORTF (5U)


/* Tiva-c Pins */
#define PIN_NO_0 (0U)
#define PIN_NO_1 (1U)
#define PIN_NO_2 (2U)
#define PIN_NO_3 (3U)
#define PIN_NO_4 (4U)
#define PIN_NO_5 (5U)
#define PIN_NO_6 (6U)
#define PIN_NO_7 (7U)





#endif /* PORT_CFG_H_ */
