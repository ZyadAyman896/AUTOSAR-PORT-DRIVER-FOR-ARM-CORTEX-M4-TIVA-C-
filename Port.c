/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Zyad Ayman
 ******************************************************************************/
#include "Det.h"
#include "Port.h"
#include "tm4c123gh6pm_registers.h"






/* Holds the status of the Port
 * options:     PORT_NOT_INITIALIZED
 *              PORT_INITIALIZED        ( set by Port_Init() )
*/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;




/************************************************************************************
* Service Name: Port_SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigType * ConfigPtr )
{
    volatile uint32 *PointerToPort = NULL_PTR; /* point to the required Port Registers base address */

    switch(ConfigPtr->port_num)
    {
        case  0: PointerToPort = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		         break;
     	case  1: PointerToPort = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		         break;
	    case  2: PointerToPort = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		         break;
	    case  3: PointerToPort = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		         break;
        case  4: PointerToPort = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		         break;
        case  5: PointerToPort = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		         break;
    }
    
    if( ((ConfigPtr->port_num == 3) && (ConfigPtr->pin_num == 7)) || ((ConfigPtr->port_num == 5) && (ConfigPtr->pin_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_COMMIT_REG_OFFSET) , ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (ConfigPtr->port_num == 2) && (ConfigPtr->pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    if(ConfigPtr->direction == OUTPUT)
    {
	    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(ConfigPtr->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(ConfigPtr->direction == INPUT)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(ConfigPtr->resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(ConfigPtr->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

    /* Setup the pin mode as GPIO */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->pin_num * 4));     /* Clear the PMCx bits for this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

}










/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: void
* Description: Initializes the Port Driver module.
************************************************************************************/




void Port_Init( const Port_ConfigType* ConfigPtr )
{
    #if (PORT_DEV_ERROR_DETECT == STD_ON)
        if(ConfigPtr == NULL)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_POINTER);
            return ;
        }
    #endif

    /* point to the required Port Registers base address */
    volatile uint32 *PointerToPort = NULL_PTR;

    Port_Status = PORT_INITIALIZED;

    uint8 counter ;

    for(counter = 0 ; counter < PORT_CONFIGURED_PINS ; counter++)
    {

        /* Putting the Port Base Register Address in the pointer */
        switch(ConfigPtr -> Pin[counter].port_num)
        {
            case  PORTA: PointerToPort = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
            case  PORTB: PointerToPort = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
            case  PORTC: PointerToPort = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
            case  PORTD: PointerToPort = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
            case  PORTE: PointerToPort = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
            case  PORTF: PointerToPort = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
        }

        /*  Enabling Clock for any Port                   */
        GPIO_vEnable_Clock(ConfigPtr ->Pin[counter].port_num);


        /*  Setting the lock and commit of the pins      */
        SetLock_Commit(PointerToPort , ConfigPtr ,counter );


        /*  Setting pin direction */
        SetPinDirection_(PointerToPort , ConfigPtr ,counter );


        /*  Setting Pin mode  */
        SetPinMode_(PointerToPort , ConfigPtr , counter);





    }



}


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

#if(PORT_SET_PIN_DIRECTION_SID == STD_ON)
void  SetPinDirection(  Port_PinType Pin,  PinDirectionType Direction )
{
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    if( Port_Status == PORT_NOT_INITIALIZED )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
    {
        /*   Do Nothing      */
    }

    if ( Pin > PORT_CONFIGURED_PINS )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
    }
    else
    {
       /*   Do Nothing      */
    }

    /* check if Port Pin not configured as changeable */
    if(ConfigPtr->Pin[Pin].pin_dir_changeable == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_DIRECTION_UNCHANGEABLE);
    }
    else
    {
        /* Do Nothing */
    }
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

    switch(ConfigPtr->Pin[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
         break;
        case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
         break;
        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
         break;
        case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
         break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
         break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
         break;
    }

    if( (ConfigPtr->Pin[Pin].port_num == 2) && (ConfigPtr->Pin[Pin].pin_num <= 3)) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
        return;
    }

    if(Direction == PIN_OUT)
    {
        /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);
    }
    else if(Direction == PIN_IN)
    {
        /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);
    }
    else
    {
        /* Do Nothing */
    }

#endif






}
#endif

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

void  RefreshPortDirection( void )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIR_SID, PORT_E_UNINIT);
    }
    else
    {   /* Do Nothing */    }
#endif


for(Port_PinType index = ZERO; index < PORT_CONFIGURED_PINS; index++)
    {
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

        switch(ConfigPtr->Pin[index].port_num)
        {
            case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
             break;
            case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
             break;
            case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
             break;
            case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
             break;
            case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
             break;
            case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
             break;
        }

        if( (ConfigPtr->Pin[index].port_num == 2) && (ConfigPtr->Pin[index].pin_num <= 3)) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
            continue;
        }

        /* PORT061: The function Port_RefreshPortDirection shall exclude those port pins from
         * refreshing that are configured as pin direction changeable during runtime */
        if (ConfigPtr->Pin[index].pin_dir_changeable == STD_OFF)
        {
            if(ConfigPtr->Pin[index].direction == PIN_OUT)
            {
                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->Pin[index].pin_num);
            }
            else if(ConfigPtr->Pin[index].direction == PIN_OUT)
            {
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->Pin[index].pin_num);
            }
            else
            {   /* Do Nothing */    }
        }
        else
        {   /* Do Nothing */    }
    }




}


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
#if (PORT_VERSION_INFO_API == STD_ON)
void GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif


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


void  SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{

#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the Driver is initialized before using this function */
        if(Port_Status == PORT_NOT_INITIALIZED)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
        }
        else
        {   /* Do Nothing */    }

        /* check if incorrect Port Pin ID passed */
        if(Pin >= PORT_CONFIGURED_PINS)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        }
        else
        {   /* Do Nothing */    }

        /* check if the Port Pin Mode passed not valid */
        if(Mode > PIN_MODE_DIO)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        }
        else
        {   /* Do Nothing */    }

        /* check if the API called when the mode is unchangeable */
        if(ConfigPtr->Pin[Pin].pin_mode_changeable == STD_OFF)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        }
        else
        {   /* Do Nothing */    }
    #endif

    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

    switch(ConfigPtr->Pin[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
         break;
        case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
         break;
        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
         break;
        case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
         break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
         break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
         break;
    }

    if( (ConfigPtr->Pin[Pin].port_num == 2) && (ConfigPtr->Pin[Pin].pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
        return;
    }

    if (Mode == PIN_MODE_DIO)
    {
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);

        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);

        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->Pin[Pin].pin_num * 4));

        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);
    }
    else if (Mode == PIN_MODE_ADC)
    {
        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);

        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);

        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->Pin[Pin].pin_num * 4));

        /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);
    }
    else /* Another mode */
    {
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);

        /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr ->Pin[Pin].pin_num);

        /* Set the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Mode & 0x0000000F << (ConfigPtr->Pin[Pin].pin_num * 4));

        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pin[Pin].pin_num);
    }



}






/*******************************************************************************
 *                              Static functions                              *
 *******************************************************************************/


/* Description :   Setting pin direction */
STATIC void SetPinDirection_(const volatile uint32 *PointerToPort_ , const Port_ConfigType* ConfigPtr_ , uint8 counter_)
{

    /*  Adding the offset for direction register  */
    PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ + PORT_DIR_REG_OFFSET) ;

    /*Configuring the direction of the pin required for the specific port */
    if (ConfigPtr_ -> Pin[counter_].direction == INPUT)
    {
        /* Setting the bit to be input */
        SET_BIT(*PointerToPort_ ,ConfigPtr_ -> Pin[counter_].pin_num ) ;

        /*  Subtracting the offset for direction register to return to point to the base address */
        PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ - PORT_DIR_REG_OFFSET) ;

        /* Configuring pin resistor type */
        if( ConfigPtr_ -> Pin[counter_].resistor == PULL_UP )
        {
            /*  Adding the offset for PORT_PULL_UP_REG_OFFSET  */
            PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ + PORT_PULL_UP_REG_OFFSET) ;

            SET_BIT(*PointerToPort_ ,ConfigPtr_ -> Pin[counter_].pin_num ) ;

            /*  Subtracting the offset for PORT_PULL_UP_REG_OFFSET to return to point to the base address */
            PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ - PORT_PULL_UP_REG_OFFSET) ;

        }
        else if ( ConfigPtr_ -> Pin[counter_].resistor == PULL_DOWN )
        {
            /*  Adding the offset for PORT_PULL_DOWN_REG_OFFSET  */
            PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ + PORT_PULL_DOWN_REG_OFFSET) ;

            SET_BIT(*PointerToPort_ ,ConfigPtr_ -> Pin[counter_].pin_num ) ;

            /*  Subtracting the offset for PORT_PULL_DOWN_REG_OFFSET to return to point to the base address */
            PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ - PORT_PULL_DOWN_REG_OFFSET) ;
        }
        else
        {
            /*  Do nothing    */
        }
    }
    else if (ConfigPtr_ -> direction == OUTPUT)
    {
        /* Setting the bit to be output */
        CLEAR_BIT(*PointerToPort_ ,ConfigPtr_  -> Pin[counter_].pin_num ) ;
    }




}




/* Description : Adjusting the lock and commit for pins    */

STATIC void SetLock_Commit(const volatile uint32  *PointerToPort_ , const Port_ConfigType* ConfigPtr_ , uint8 counter_)
{

    if( ( (ConfigPtr_->Pin[counter_].port_num == 3) && (ConfigPtr_->Pin[counter_].pin_num == 7) ) || ( (ConfigPtr_->Pin[counter_].port_num == 5) && (ConfigPtr_->Pin[counter_].pin_num == 0) ) )
    {

        /*  Adding the offset for GPIO_PORT_LOCK_REG  */
        PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ + PORT_LOCK_REG_OFFSET) ;


        /* Unlock the GPIO_PORT_LOCK_REG */
        WRITE_REG(*PointerToPort_ ,PORT_MAGIC_LOCK_NUMBER);

        /* Subtracting the offset for GPIO_PORT_LOCK_REG to return to point to the base address */
        PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ - PORT_LOCK_REG_OFFSET) ;

        /* Adding the offset for PORT_COMMIT_REG_OFFSET  */
        PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ + PORT_COMMIT_REG_OFFSET) ;

        /* Enable changes on PXn  for commit register*/
        SET_PIN(*PointerToPort_ ,copy_uint8PinNumber);

        /* Subtracting the offset for PORT_COMMIT_REG_OFFSET to return to point to the base address */
        PointerToPort_ = (volatile uint32 *)((uint8 *)PointerToPort_ - PORT_COMMIT_REG_OFFSET) ;

    }
    /*  Checking if they are JTAG Pins   */
    else if ( ( ConfigPtr_->Pin[counter_].port_num == 2 ) && ( ConfigPtr_->Pin[counter_].pin_num <= 3 ) )
    {

        /* Do Nothing */

    }
    else
    {



    }



}

/* Description : Enabling Clock for any Port */
STATIC void GPIO_vEnable_Clock(uint8 copy_uint8PortName)
{

    switch (copy_uint8PortName)
    {
        case PORTA :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_0);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_0));
        break ;

        case PORTB :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_1);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_1));
        break ;

        case PORTC :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_2);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_2));
        break ;

        case PORTD :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_3);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_3));
        break ;

        case PORTE :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_4);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_4));
        break ;

        case PORTF :
            /*ACTIVATING THE CLOCK*/
            SET_PIN(SYSCTL_RCGCGPIO_REG , PIN_NO_5);
            /*WAITING FOR THE CLOCK TO BE ACTIVATED AFTER 3 CLOCK CYCLES*/
            while(!READ_PIN(SYSCTL_PRGPIO_REG, PIN_NO_5));
        break ;
    }

}



/*


typedef struct
{
    uint8 port_num;
    uint8 pin_num;
    uint8 pin_mode;
    PinDirection direction;
    InternalResistor resistor;
    PinLevelValue initial_value;
    PinInitialMode initial_mode;    /* e.g.  PIN_MODE_DIO
    uint8 pin_dir_changeable;           /* STD_ON,STD_OFF
    uint8 pin_mode_changeable;          /* STD_ON,STD_OFF
} Port_ConfigType;


/*  Setting Pin mode  */
STATIC void SetPinMode_(const volatile uint32  *PointerToPort_  , const Port_ConfigType* ConfigPtr_ , uint8 counter_)
{
        if (ConfigPtr_->Pin[counter_].initial_mode == PIN_MODE_DIO)
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr_->Pin[counter_].pin_num * 4));

                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);
            }
            else if (ConfigPtr_->Pin[counter_].initial_mode == PIN_MODE_ADC)
            {
                /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Clear the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr_->Pin[counter_].pin_num * 4));

                /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);
            }
            else /* Another mode */
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);

                /* Set the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_CTL_REG_OFFSET) |= (ConfigPtr_->Pin[counter_].initial_mode & 0x0000000F << (ConfigPtr_->Pin[counter_].pin_num * 4));

                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PointerToPort_ + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr_->Pin[counter_].pin_num);
            }
}

