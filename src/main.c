/*==================================================================================================
*   Project              : RTD AUTOSAR 4.7
*   Platform             : CORTEXM
*   Peripheral           : S32K3XX
*   Dependencies         : none
*
*   Autosar Version      : 4.9.0
*   Autosar Revision     : ASR_REL_4_9_REV_0000
*   Autosar Conf.Variant :
*   SW Version           : 7.0.0
*   Build Version        : S32K3_RTD_7_0_0_QLP03_D2512_ASR_REL_4_9_REV_0000_20251210
*
*   Copyright 2020 - 2025 NXP
*
*   NXP Proprietary. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms.  By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms.  If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
 * @file    main.c
 * @brief   WKPU Standby Wakeup Demo with SSD1306 OLED Display - S32K344
 *
 * @details This application demonstrates how to use the WKPU (Wakeup Unit)
 *          peripheral on the S32K344 MCU to enter and exit Standby low-power
 *          mode. An SSD1306 OLED display (driven via FlexIO I2C) provides
 *          visual feedback of the current system state:
 *
 *          - On cold boot:  displays "Hello from S32K344!"
 *          - On wakeup:     displays "Woken up!" alert
 *          - On SW2 press:  displays "Entering Standby..." then powers down
 *
 *          Hardware requirements:
 *          - FRDM-S32K344 evaluation board (or compatible)
 *          - SSD1306 128x64 OLED display connected via FlexIO I2C
 *              - SCL: PTA14 (FXIO D14)
 *              - SDA: PTA9  (FXIO D7)
 *          - Reed switch or push-button on WKPU channel 18 (PTB26)
 *          - User button SW2 for standby entry
 *          - Green LED (PTA29) for run/sleep status indication
 *
 *          Wakeup modes:
 *          - WKPU_USE_FAST = 1 : Fast standby exit (core resets via
 *            FastWkpuBootVectorTable, SRAM partially retained)
 *          - WKPU_USE_FAST = 0 : Normal standby exit (full power-on reset
 *            sequence, execution resumes after WFI)
 *
 * @note    On fast wakeup the core performs a functional reset, so execution
 *          always restarts from the top of main(). The reset reason register
 *          is used to distinguish a cold boot from a standby wakeup.
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "Clock_Ip.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Flexio_Mcl_Ip.h"
#include "Flexio_I2c_Ip.h"
#include "OsIf.h"
#include "Power_Ip.h"
#include "Wkpu_Ip.h"
#include "ssd1306.h"
#include <string.h>

/*==================================================================================================
*                                          DEFINES
==================================================================================================*/
/**
 * @brief   Wakeup mode selector.
 *          Set to 1 for fast standby exit (preserves standby SRAM, faster boot).
 *          Set to 0 for normal standby exit (full POR sequence).
 */
#define WKPU_USE_FAST       1U

/** @brief  WKPU hardware instance index. */
#define WKPU_INST           0U

/**
 * @brief   Reset reason value indicating a wakeup from standby.
 * @details Matches the McuResetReasonConf entry "MCU_WAKEUP_REASON"
 *          configured in the .mex Peripherals tool (index 28).
 */
#define MCU_WAKEUP_REASON   ((Power_Ip_ResetType)28U)

/** @brief  Duration in milliseconds to show OLED messages before proceeding. */
#define OLED_MSG_DISPLAY_MS 2000U

/** @brief  Duration in milliseconds to show the standby countdown message. */
#define OLED_STANDBY_MS     1500U

/*==================================================================================================
*                                     EXTERN DECLARATIONS
==================================================================================================*/
/** @brief  Default fault handler defined in startup code (startup_cm7.s). */
extern void undefined_handler(void);

/*==================================================================================================
*                                  FORWARD DECLARATIONS
==================================================================================================*/
static void Wkpu_FastWkpuBootAddress(void);
static void Wkpu_Config(void);
static void Wkpu_EnterStandby(void);
static void OLED_Init(void);
static void OLED_ShowMessage(const char *line1, const char *line2, uint32 delayMs);
static void DelayMs(uint32 ms);

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
/**
 * @brief   Stack pointer value loaded by the core on fast standby wakeup.
 * @details Points to the top of the 32 KB standby-retained SRAM region.
 *          This gives the early wakeup handler a valid stack before
 *          Reset_Handler reinitialises SP from the main vector table.
 */
static const uint32 Standby_Stack_StartAddr = 0x20408000U;

/**
 * @brief   Fast-Standby Boot Vector Table.
 *
 * @details This mini vector table is placed in flash at the address
 *          configured in McuBootBaseAddress (DCM_GPR) via the .mex file
 *          (default 0x00402800).  The linker script pins the
 *          .fast_wkpu_boot_vector section to that exact address.
 *
 *          On fast standby wakeup the Cortex-M7 core:
 *            [0] Loads SP  <- Standby_Stack_StartAddr
 *            [1] Branches  <- Wkpu_FastWkpuBootAddress (fixes FIRC, then
 *                             jumps to Reset_Handler for full init)
 *            [2] Fault     <- undefined_handler (safety catch)
 */
__attribute__((section(".fast_wkpu_boot_vector"), aligned(1024)))
uintptr_t const FastWkpuBootVectorTable[] =
{
    (const uintptr_t)Standby_Stack_StartAddr,
    (const uintptr_t)&Wkpu_FastWkpuBootAddress,
    (const uintptr_t)&undefined_handler,
};

/*==================================================================================================
*                                      LOCAL FUNCTIONS
==================================================================================================*/

/**
 * @brief   Millisecond busy-wait delay using OsIf elapsed-counter API.
 * @param[in] ms  Number of milliseconds to wait.
 */
static void DelayMs(uint32 ms)
{
    uint32 cur     = OsIf_GetCounter(OSIF_COUNTER_SYSTEM);
    uint32 elapsed = 0U;
    uint32 timeout = OsIf_MicrosToTicks(ms * 1000U, OSIF_COUNTER_SYSTEM);

    while (elapsed < timeout)
    {
        elapsed += OsIf_GetElapsed(&cur, OSIF_COUNTER_SYSTEM);
    }
}

/**
 * @brief   Early wakeup handler executed before Reset_Handler.
 *
 * @details After fast standby wakeup the FIRC oscillator runs at 24 MHz
 *          (hardware default).  The NXP startup code and clock driver
 *          expect FIRC at 48 MHz, so this function reprograms the FIRC
 *          divider via a direct register write (no driver dependencies)
 *          and then branches to Reset_Handler for full C-runtime and
 *          peripheral reinitialisation.
 *
 * @note    This function runs with a minimal stack (Standby_Stack_StartAddr)
 *          and must NOT call any C library or driver functions.
 */
static void Wkpu_FastWkpuBootAddress(void)
{
    if (((IP_CONFIGURATION_GPR->CONFIG_REG_GPR
          & CONFIGURATION_GPR_CONFIG_REG_GPR_APP_CORE_ACC_MASK)
         >> CONFIGURATION_GPR_CONFIG_REG_GPR_APP_CORE_ACC_SHIFT) == 5U)
    {
        IP_CONFIGURATION_GPR->CONFIG_REG_GPR =
            CONFIGURATION_GPR_CONFIG_REG_GPR_APP_CORE_ACC(5U)
            | CONFIGURATION_GPR_CONFIG_REG_GPR_FIRC_DIV_SEL(3U);
    }

    __asm("bl Reset_Handler");
}

/**
 * @brief   Arm the WKPU channel for standby wakeup.
 */
static void Wkpu_Config(void)
{
    Wkpu_Ip_Init(WKPU_INST, &Wkpu_Ip_Config_PB);
    Wkpu_Ip_EnableInterrupt(WKPU_INST, Wkpu_Ip_ChannelConfig_PB[0].hwChannel);
}

/**
 * @brief   Prepare the MCU and enter Standby mode.
 */
static void Wkpu_EnterStandby(void)
{
    Clock_Ip_Init(&Clock_Ip_aClockConfig[1]);

    Wkpu_Config();

#if (1U == WKPU_USE_FAST)
    Power_Ip_SetMode(&Power_Ip_aModeConfigPB[2]);
#else
    Power_Ip_SetMode(&Power_Ip_aModeConfigPB[1]);
#endif
}

/**
 * @brief   Initialise the FlexIO I2C bus and SSD1306 OLED controller.
 */
static void OLED_Init(void)
{
    Flexio_Mcl_Ip_Init(Flexio_Ip_paxBase[0]);
    Flexio_Mcl_Ip_InitDevice(&Flexio_Ip_Sa_xFlexioInit);

    Flexio_I2c_Ip_MasterInit(OLED_FLEXIO_INSTANCE, OLED_FLEXIO_CHANNEL,
                             &Flexio_I2cMasterChannel0);

    SSD1306_Init();
}

/**
 * @brief   Display a two-line message on the OLED and hold for a delay.
 *
 * @param[in] line1     First line of text (displayed at y=10). NULL to skip.
 * @param[in] line2     Second line of text (displayed at y=30). NULL to skip.
 * @param[in] delayMs   Milliseconds to keep the message visible. 0 = no wait.
 */
static void OLED_ShowMessage(const char *line1, const char *line2, uint32 delayMs)
{
    SSD1306_ClearBuffer();

    if ((line1 != NULL_PTR) && (line1[0] != '\0'))
    {
        SSD1306_DrawString(4, 10, line1, 1, true);
    }
    if ((line2 != NULL_PTR) && (line2[0] != '\0'))
    {
        SSD1306_DrawString(4, 30, line2, 1, true);
    }

    SSD1306_UpdateScreen();

    if (delayMs > 0U)
    {
        DelayMs(delayMs);
    }
}

/*==================================================================================================
*                                          MAIN
==================================================================================================*/
int main(void)
{
    Clock_Ip_StatusType clockStatus = CLOCK_IP_ERROR;
    boolean isWakeup = (boolean)FALSE;

    /* 1. Initialise the full clock tree (PLL, FXOSC, dividers) */
    while ((clockStatus = Clock_Ip_Init(&Clock_Ip_aClockConfig[0])) != CLOCK_IP_SUCCESS)
    {
    }

    /* 2. Initialise MC_RGM, PMC and release padkeeping */
    Power_Ip_Init(&Power_Ip_HwIPsConfigPB);

    /* 3. Detect reset reason - distinguish cold boot from standby wakeup.
     *    Also clears the RDSS flag to prevent re-entry into standby. */
    Power_Ip_ResetType resetReason = Power_Ip_GetResetReason();

    if (resetReason == MCU_WAKEUP_REASON)
    {
        isWakeup = (boolean)TRUE;
    }

    /* 4. Set RUN mode - enables peripheral partition clocks.
     *    MUST be done before accessing any peripheral. */
    Power_Ip_SetMode(&Power_Ip_aModeConfigPB[0]);

    /* 5. Initialise OS Interface (provides OsIf counter/tick services) */
    OsIf_Init(NULL_PTR);

    /* 6. Configure pin multiplexing */
    Siul2_Port_Ip_Init(
        NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
        g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* 7. Initialise WKPU (channel armed later in Wkpu_EnterStandby) */
    Wkpu_Ip_Init(WKPU_INST, &Wkpu_Ip_Config_PB);

    /* 8. Initialise FlexIO I2C and the SSD1306 OLED display */
    OLED_Init();

    /* 9. Show boot message depending on reset reason */
    Siul2_Dio_Ip_WritePin(LED_GRN_PORT, LED_GRN_PIN, 1U);

    if (isWakeup)
    {
        OLED_ShowMessage("** WAKEUP! **", "From Standby", OLED_MSG_DISPLAY_MS);
    }
    else
    {
        OLED_ShowMessage("Hello from", "S32K344!", OLED_MSG_DISPLAY_MS);
    }

    /* Show idle screen */
    OLED_ShowMessage("System Ready", "SW2: Standby", 0U);

    /* 10. Main loop */
    for (;;)
    {
        if (Siul2_Dio_Ip_ReadPin(SW2_PORT, SW2_PIN) == 1U)
        {
            /* Debounce: wait for release */
            while (Siul2_Dio_Ip_ReadPin(SW2_PORT, SW2_PIN) == 1U)
            {
            }

            /* LED off */
            Siul2_Dio_Ip_WritePin(LED_GRN_PORT, LED_GRN_PIN, 0U);

            /* Show standby message */
            OLED_ShowMessage("Entering", "Standby...", OLED_STANDBY_MS);

            /* Blank display before power-down */
            SSD1306_ClearBuffer();
            SSD1306_UpdateScreen();

            /* Enter Standby (FAST mode: never returns, resets to main) */
            Wkpu_EnterStandby();

            /* Reached only in NORMAL wakeup mode */
            Siul2_Dio_Ip_WritePin(LED_GRN_PORT, LED_GRN_PIN, 1U);
            OLED_ShowMessage("* WAKEUP! *", "Normal exit", OLED_MSG_DISPLAY_MS);
            OLED_ShowMessage("System Ready", "SW2: Standby", 0U);
        }
    }

    return 0;
}
