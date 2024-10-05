 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Zyad Ayman
 ******************************************************************************/

#ifndef  PORT_H
#define  PORT_H



/* Id for the company in the AUTOSAR */
#define  VENDOR_ID              (1000U)

/* PORT Module Id */
#define  PORT_MODULE_ID              (140U)

/* PORT Instance Id */
#define  PORT_INSTANCE_ID             (0U)

/*
 * Module Version 1.0.0
 */
#define  PORT_SW_MAJOR_VERSION           (1U)
#define  PORT_SW_MINOR_VERSION           (0U)
#define  PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define  PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define  PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define  PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT_ Status
 */
#define  PORT_INITIALIZED                (1U)
#define  PORT_NOT_INITIALIZED            (0U)



/* Standard AUTOSAR types */
#include "Std_Types.h"


/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION !=  PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION !=  PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION !=  PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between  Cfg.h and Port.h files */
#if (( PORT_CFG_AR_RELEASE_MAJOR_VERSION !=  PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  ( PORT_CFG_AR_RELEASE_MINOR_VERSION !=  PORT_AR_RELEASE_MINOR_VERSION)\
 ||  ( PORT_CFG_AR_RELEASE_PATCH_VERSION !=  PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of  Cfg.h does not match the expected version"
#endif

/* Software Version checking between  Cfg.h and Port.h files */
#if (( PORT_CFG_SW_MAJOR_VERSION !=  PORT_SW_MAJOR_VERSION)\
 ||  ( PORT_CFG_SW_MINOR_VERSION !=  PORT_SW_MINOR_VERSION)\
 ||  ( PORT_CFG_SW_PATCH_VERSION !=  PORT_SW_PATCH_VERSION))
  #error "The SW version of  Cfg.h does not match the expected version"
#endif


/* Non AUTOSAR files */
#include "Common_Macros.h"


/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/*  Service ID for  INIT_SID     */
#define  PORT_INIT_SID                                   (uint8)0x00

/*  Service ID for   SET_PIN_DIRECTION_SID   */
#define  PORT_SET_PIN_DIRECTION_SID                      (uint8)0x01

/*  Service ID for  REFRESH_ DIRECTION_SID   */
#define  PORT_REFRESH_ DIRECTION_SID                     (uint8)0x02

/*  Service ID for   GET_VERSION_INFO_SID    */
#define  PORT_GET_VERSION_INFO_SID                       (uint8)0X03

/*  Service ID for   SET_PIN_MODE_SID    */
#define  PORT_SET_PIN_MODE_SID                           (uint8)0X04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/*  Service ID for  E_PARAM_PIN */
#define  PORT_E_PARAM_PIN                                (uint8)0x0A

/*  Service ID for  E_DIRECTION_UNCHANGEABLE */
#define  PORT_E_DIRECTION_UNCHANGEABLE                   (uint8)0x0B

/*  Service ID for  E_PARAM_CONFIG */
#define  PORT_E_PARAM_CONFIG                             (uint8)0x0C

/*  Service ID for  E_PARAM_INVALID_MODE */
#define  PORT_E_PARAM_INVALID_MODE                       (uint8)0x0D

/*  Service ID for  E_MODE_UNCHANGEABLE */
#define  PORT_E_MODE_UNCHANGEABLE                        (uint8)0x0E

/*  Service ID for  E_UNINIT */
#define  PORT_E_UNINIT                                   (uint8)0x0F

/*  Service ID for  E_PARAM_POINTER */
#define  PORT_E_PARAM_POINTER                            (uint8)0x10


/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/



/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define  DATA_REG_OFFSET              0x3FC
#define  DIR_REG_OFFSET               0x400
#define  ALT_FUNC_REG_OFFSET          0x420
#define  PULL_UP_REG_OFFSET           0x510
#define  PULL_DOWN_REG_OFFSET         0x514
#define  DIGITAL_ENABLE_REG_OFFSET    0x51C
#define  LOCK_REG_OFFSET              0x520
#define  COMMIT_REG_OFFSET            0x524
#define  ANALOG_MODE_SEL_REG_OFFSET   0x528
#define  CTL_REG_OFFSET               0x52C

/*USED TO UNLOCK THE PORT BY ASSIGNING IT IN LOCK REGISTER*/

#define  MAGIC_LOCK_NUMBER 0x4C4F434B


/* RCC Registers */
#define SYSCTL_REGCGC2_REG              (*((volatile uint32 *)0x400FE108))

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT,
    OUTPUT
} PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,
    PULL_UP,
    PULL_DOWN
} InternalResistor;


/* Description : Data type for the symbolic name of a port pin.    */
typedef uint8  Port_PinType ;


/*  Description: Possible directions of a port pin.  */
typedef enum
{
     PIN_IN,
     PIN_OUT

} PinDirectionType;

/* Description: Port Pin Level value from Port pin list */

typedef enum
{
    PIN_LEVEL_LOW,
    PIN_LEVEL_HIGH

} PinLevelValue;


/*   Description: Port pin mode from mode list for use with  Init() function*/
typedef enum
{
     PIN_MODE_ADC,
     PIN_MODE_ALT1,
     PIN_MODE_ALT2,
     PIN_MODE_ALT3,
     PIN_MODE_ALT4,
     PIN_MODE_ALT5,
     PIN_MODE_ALT6,
     PIN_MODE_ALT7,
     PIN_MODE_ALT8,
     PIN_MODE_ALT9,
     PIN_MODE_DIO,

} PinInitialMode;


/*  Description: Different port pin modes.  */
typedef uint8 Port_PinModeType





/* Description: Structure to configure each individual PIN:
 *  1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *  2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *  5. initial port pin value (high, low)
 *  6. initial port pin mode  (ADC, DIO, ..)
 *  7. pin direction changeable (true, false)
 *  8. pin mode changeable (true, false)
 */

typedef struct
{
    uint8 port_num;
    uint8 pin_num;
    uint8 pin_mode;
    PinDirection direction;
    InternalResistor resistor;
    PinLevelValue initial_value;
    PinInitialMode initial_mode;    /* e.g.  PIN_MODE_DIO                   */
    uint8 pin_dir_changeable;           /* STD_ON,STD_OFF                           */
    uint8 pin_mode_changeable;          /* STD_ON,STD_OFF
} Port_ConfigType;


/*   Description: Array of  ConfigPin */

typedef struct
{
     ConfigPin Pin[CONFIGURED_PINS];
} Port_ConfigType;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/



/************************************************************************************
* Service Name:  SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/


void  SetupGpioPin(const  ConfigType *ConfigPtr );


/************************************************************************************
* Service Name:  Port_Init
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/


void  Port_Init( const  ConfigType* ConfigPtr ) ;


/************************************************************************************
* Service Name:  SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in):  PinType Pin,  PinDirectionType Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/


void  SetPinDirection(  Port_PinType Pin,  PinDirectionType Direction );


/************************************************************************************
* Service Name:  RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non-Reentrant
* Parameters (in): void
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
************************************************************************************/

void  RefreshPortDirection( void );


/************************************************************************************
* Service Name:  GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non-Reentrant
* Parameters (in):  Std_VersionInfoType* versioninfo
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/

void  GetVersionInfo( Std_VersionInfoType* versioninfo );


/************************************************************************************
* Service Name:  SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Non-Reentrant
* Parameters (in):  Std_VersionInfoType* versioninfo
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
************************************************************************************/


void  SetPinMode( Port_PinType Pin, Port_PinModeType Mode );



/*******************************************************************************
 *                              Static functions                              *
 *******************************************************************************/


/* Description :   Setting pin direction                   */
STATIC void SetPinDirection_(const volatile uint32 *PointerToPort_  , const Port_ConfigType* ConfigPtr_ , uint8 counter_);


/* Description : Adjusting the lock and commit for pins    */
STATIC void SetLock_Commit(const volatile uint32 *PointerToPort_ );

/* Description : Enabling Clock for any Port               */
STATIC void GPIO_vEnable_Clock(uint8 copy_uint8PortName);


/* Description : Setting Pin mode                          */
STATIC void SetPinMode_(const volatile uint32  *PointerToPort_  , const Port_ConfigType* ConfigPtr_ , uint8 counter_)




/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Post build structure used with Port_Init API */
extern const Port_ConfigType Port_PinConfig;




#endif /*  H */
