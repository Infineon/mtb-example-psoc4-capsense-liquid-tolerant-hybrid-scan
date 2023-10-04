/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the MSC CAPSENSE liquid tolerant 
* hybrid scan Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#define CY_ASSERT_FAILED                 (0u)
#define MSC_CAPSENSE_WIDGET_INACTIVE     (0u)
#define LED_ON                           (0u)
#define LED_OFF                          (1u)

/* EZI2C interrupt priority must be higher than CapSense interrupt */
#define EZI2C_INTR_PRIORITY              (2u)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* EZI2C slave context structure */
cy_stc_scb_ezi2c_context_t ezi2c_context;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
static void led_control(void);
void start_sample_callback_function(cy_stc_active_scan_sns_t * ptrActiveScan);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - scan touch input continuously
*
* Return:
*  int
*
* Parameters:
*  void
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize MSC CapSense */
    initialize_capsense();

    /* Register the Start_Sample callback for CapSense */
    Cy_CapSense_RegisterCallback(CY_CAPSENSE_START_SAMPLE_E,
        start_sample_callback_function,
        &cy_capsense_context);

    /* Initializes the baselines of all the sensors of all the widgets */
    Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);

    /* Start the first scan */
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Turns LED ON/OFF based on button status */
            led_control();

            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Start the next scan */
            Cy_CapSense_ScanAllSlots(&cy_capsense_context);
        }
    }
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense blocks and configures the CapSense
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* CapSense initialization failed, the middleware may not operate
         * as expected, and repeating of initialization is required.*/
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the README.md file */
    }
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: capsense_msc1_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC1 block.
 *
 *******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CapSense Tuner tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize the interrupt */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    
    /* Enable the interrupt */
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
        sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
        &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);
}

/*******************************************************************************
 * Function Name: led_control
 ********************************************************************************
 * Summary:
 *  Turning LED ON/OFF based on button status
 *
 *******************************************************************************/
static void led_control(void)
{
    if((MSC_CAPSENSE_WIDGET_INACTIVE != Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context)) && (MSC_CAPSENSE_WIDGET_INACTIVE == Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context)))
    {
        Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, CYBSP_LED_STATE_ON);  /* LED2 ON */
        Cy_GPIO_Write(CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_NUM, CYBSP_LED_STATE_OFF); /* LED3 OFF */
    }
    else  if((MSC_CAPSENSE_WIDGET_INACTIVE != Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context)) && (MSC_CAPSENSE_WIDGET_INACTIVE == Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context)))
    {
        Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, CYBSP_LED_STATE_OFF); /* LED2 OFF */
        Cy_GPIO_Write(CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_NUM, CYBSP_LED_STATE_ON);  /* LED3 ON */
    }
    else if(MSC_CAPSENSE_WIDGET_INACTIVE != (Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context) && Cy_CapSense_IsWidgetActive
        (CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context)))
    {
        Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, CYBSP_LED_STATE_ON); /* LED2 ON */
        Cy_GPIO_Write(CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_NUM, CYBSP_LED_STATE_ON); /* LED3 ON */
    }
    else
    {
        Cy_GPIO_Write(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_NUM, CYBSP_LED_STATE_OFF); /* LED2 OFF */
        Cy_GPIO_Write(CYBSP_USER_LED3_PORT, CYBSP_USER_LED3_NUM, CYBSP_LED_STATE_OFF); /* LED3 OFF */
    }
}

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
 * Function Name: start_sample_callback_function
 ********************************************************************************
 * Summary:
 *  Callback function for the CAPSENSE_START_SAMPLE event. The callback will be
 *  executed before each sensor scan triggering.
 *  The pin state of the button sensors are changed in this function. The
 *  sensor adjacent to the sensor being scanned is connected to shield.
 *  Otherwise sensor is connected to ground.
 *
 * Parameter:
 *  cy_stc_active_scan_sns_t: Pointer to the current active sensor structure
 *
 *******************************************************************************/
void start_sample_callback_function(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    /* Process widgets only if the current widget being scanned is the button */
    if(ptrActiveScan->widgetIndex == CY_CAPSENSE_BUTTON0_WDGT_ID)
    {
        /* If the sensor is adjacent to the sensor being scanned,
         * configure it as shield.
         */
        Cy_CapSense_SetPinState(CY_CAPSENSE_BUTTON0_WDGT_ID,
            CY_CAPSENSE_BUTTON0_TX0_ID, CY_CAPSENSE_SHIELD,
            &cy_capsense_context);
    }
    else
    {
        /* If the sensor is not adjacent to the sensor being
         * scanned, connect it to ground.
         */
        Cy_CapSense_SetPinState(CY_CAPSENSE_BUTTON0_WDGT_ID,
            CY_CAPSENSE_BUTTON0_TX0_ID, CY_CAPSENSE_GROUND,
            &cy_capsense_context);
    }

    /* Process widgets only if the current widget being scanned is the button */
    if(ptrActiveScan->widgetIndex == CY_CAPSENSE_BUTTON1_WDGT_ID)
    {
        /* If the sensor is adjacent to the sensor being scanned,
         * configure it as shield.
         */
        Cy_CapSense_SetPinState(CY_CAPSENSE_BUTTON1_WDGT_ID,
            CY_CAPSENSE_BUTTON1_TX0_ID, CY_CAPSENSE_SHIELD,
            &cy_capsense_context);
    }
    else
    {
        /* If the sensor is not adjacent to the sensor being
         * scanned, connect it to ground.
         */
        Cy_CapSense_SetPinState(CY_CAPSENSE_BUTTON1_WDGT_ID,
            CY_CAPSENSE_BUTTON1_TX0_ID, CY_CAPSENSE_GROUND,
            &cy_capsense_context);
    }
}


/* [] END OF FILE */
