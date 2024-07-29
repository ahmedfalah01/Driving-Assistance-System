/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : uart_pal1.c
**     Project     : Driving_Assistance_System_NXP_Z2_2
**     Processor   : MPC5748G_324
**     Component   : uart_pal
**     Version     : Component SDK_S32_PA_11, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32_PA_11
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-07-01, 21:47, # CodeGen: 19
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file uart_pal1.c
** @version 01.00
*/         
/*!
**  @addtogroup uart_pal1_module uart_pal1 module documentation
**  @{
*/         
#include "uart_pal1.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files in application code.
 *
 */


/* Instance structure is used for storing PAL instance information. */
const uart_instance_t uart_pal1_instance = {
    .instType          = UART_INST_TYPE_LINFLEXD,
    .instIdx           = 10U
};

/* Serial User Configurations */
const uart_user_config_t uart_pal1_Config0 = {
    .baudRate         = 9600,
    .bitCount         = UART_8_BITS_PER_CHAR,
    .parityMode       = UART_PARITY_DISABLED,
    .stopBitCount     = UART_ONE_STOP_BIT,
    .transferType     = UART_USING_INTERRUPTS,
    .rxDMAChannel     = 0U,
    .txDMAChannel     = 0U,
    .rxCallback       = NULL,
    .rxCallbackParam  = NULL,
    .txCallback       = NULL,
    .txCallbackParam  = NULL,
    .extension        = NULL
};

/* END uart_pal1. */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP C55 series of microcontrollers.
**
** ###################################################################
*/
