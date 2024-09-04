/**
\addtogroup MyApplication MyApplication
@{
*/

/**
\file MyApplication.c
\brief Implementation

\version 1.0.0.11
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#include "applInterface.h"

#define _MY_APPLICATION_ 1
#include "MyApplication.h"
#undef _MY_APPLICATION_
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

#if COE_SUPPORTED
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
   
    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

#else
#if _WIN32
   #pragma message ("Warning: Define 'InputSize' and 'OutputSize'.")
#else
    #warning "Define 'InputSize' and 'OutputSize'."
#endif
#endif

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data

\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT8* pTmpData = (UINT8*)pData;

    /* we go through all entries of the TxPDO Assign object to get the assigned TxPDOs */
    for (j = 0; j < sTxPDOassign.u16SubIndex0; j++) {
        switch (sTxPDOassign.aEntries[j]) {
        /* TxPDO 1 */
        case 0x1A00:
            memcpy(pTmpData, &Error_code_00x603F, sizeof(Error_code_00x603F));
            pTmpData += sizeof(Error_code_00x603F);

            memcpy(pTmpData, &Statusword_00x6041, sizeof(Statusword_00x6041));
            pTmpData += sizeof(Statusword_00x6041);

            memcpy(pTmpData, &Modes_of_operation_display_00x6061, sizeof(Modes_of_operation_display_00x6061));
            pTmpData += sizeof(Modes_of_operation_display_00x6061);

            memcpy(pTmpData, &Position_actual_value_00x6064, sizeof(Position_actual_value_00x6064));
            pTmpData += sizeof(Position_actual_value_00x6064);

            memcpy(pTmpData, &Touch_probe_status_00x60B9, sizeof(Touch_probe_status_00x60B9));
            pTmpData += sizeof(Touch_probe_status_00x60B9);

            memcpy(pTmpData, &Touch_probe_pos1_pos_value_00x60BA, sizeof(Touch_probe_pos1_pos_value_00x60BA));
            pTmpData += sizeof(Touch_probe_pos1_pos_value_00x60BA);

            memcpy(pTmpData, &Following_error_actual_value_00x60F4, sizeof(Following_error_actual_value_00x60F4));
            pTmpData += sizeof(Following_error_actual_value_00x60F4);

            memcpy(pTmpData, &Digital_inputs_00x60FD, sizeof(Digital_inputs_00x60FD));
            pTmpData += sizeof(Digital_inputs_00x60FD);

            break;
        /* TxPDO 3 */
        case 0x1A01:
            memcpy(pTmpData, &Error_code_00x603F, sizeof(Error_code_00x603F));
            pTmpData += sizeof(Error_code_00x603F);

            memcpy(pTmpData, &Statusword_00x6041, sizeof(Statusword_00x6041));
            pTmpData += sizeof(Statusword_00x6041);

            memcpy(pTmpData, &Modes_of_operation_display_00x6061, sizeof(Modes_of_operation_display_00x6061));
            pTmpData += sizeof(Modes_of_operation_display_00x6061);

            memcpy(pTmpData, &Position_actual_value_00x6064, sizeof(Position_actual_value_00x6064));
            pTmpData += sizeof(Position_actual_value_00x6064);

            memcpy(pTmpData, &Velocity_actual_value_00x606C, sizeof(Velocity_actual_value_00x606C));
            pTmpData += sizeof(Velocity_actual_value_00x606C);

            memcpy(pTmpData, &Torque_actual_value_00x6077, sizeof(Torque_actual_value_00x6077));
            pTmpData += sizeof(Torque_actual_value_00x6077);

            memcpy(pTmpData, &Touch_probe_status_00x60B9, sizeof(Touch_probe_status_00x60B9));
            pTmpData += sizeof(Touch_probe_status_00x60B9);

            memcpy(pTmpData, &Touch_probe_pos1_pos_value_00x60BA, sizeof(Touch_probe_pos1_pos_value_00x60BA));
            pTmpData += sizeof(Touch_probe_pos1_pos_value_00x60BA);

            memcpy(pTmpData, &Digital_inputs_00x60FD, sizeof(Digital_inputs_00x60FD));
            pTmpData += sizeof(Digital_inputs_00x60FD);

            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT8* pTmpData = (UINT8*)pData;

    /* we go through all entries of the RxPDO Assign object to get the assigned RxPDOs */
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++) {
        switch (sRxPDOassign.aEntries[j]) {
        /* RxPDO 2 */
        case 0x1600:
            memcpy(&Controlword_00x6040, pTmpData, sizeof(Controlword_00x6040));
            pTmpData += sizeof(Controlword_00x6040);

            memcpy(&Modes_of_operation_00x6060, pTmpData, sizeof(Modes_of_operation_00x6060));
            pTmpData += sizeof(Modes_of_operation_00x6060);

            memcpy(&Target_position_00x607A, pTmpData, sizeof(Target_position_00x607A));
            pTmpData += sizeof(Target_position_00x607A);

            memcpy(&Touch_probe_function_00x60B8, pTmpData, sizeof(Touch_probe_function_00x60B8));
            pTmpData += sizeof(Touch_probe_function_00x60B8);

            break;
        case 0x1601:
            memcpy(&Controlword_00x6040, pTmpData, sizeof(Controlword_00x6040));
            pTmpData += sizeof(Controlword_00x6040);

            memcpy(&Modes_of_operation_00x6060, pTmpData, sizeof(Modes_of_operation_00x6060));
            pTmpData += sizeof(Modes_of_operation_00x6060);

            memcpy(&Target_torque_00x6071, pTmpData, sizeof(Target_torque_00x6071));
            pTmpData += sizeof(Target_torque_00x6071);

            memcpy(&Target_position_00x607A, pTmpData, sizeof(Target_position_00x607A));
            pTmpData += sizeof(Target_position_00x607A);

            memcpy(&Max_motor_speed_00x6080, pTmpData, sizeof(Max_motor_speed_00x6080));
            pTmpData += sizeof(Max_motor_speed_00x6080);

            memcpy(&Touch_probe_function_00x60B8, pTmpData, sizeof(Touch_probe_function_00x60B8));
            pTmpData += sizeof(Touch_probe_function_00x60B8);

            memcpy(&Target_velocity_00x60FF, pTmpData, sizeof(Target_velocity_00x60FF));
            pTmpData += sizeof(Target_velocity_00x60FF);

            break;
        case 0x1602:
            memcpy(&Controlword_00x6040, pTmpData, sizeof(Controlword_00x6040));
            pTmpData += sizeof(Controlword_00x6040);

            memcpy(&Modes_of_operation_00x6060, pTmpData, sizeof(Modes_of_operation_00x6060));
            pTmpData += sizeof(Modes_of_operation_00x6060);

            memcpy(&Max_torque_00x6072, pTmpData, sizeof(Max_torque_00x6072));
            pTmpData += sizeof(Max_torque_00x6072);

            memcpy(&Target_position_00x607A, pTmpData, sizeof(Target_position_00x607A));
            pTmpData += sizeof(Target_position_00x607A);

            memcpy(&Touch_probe_function_00x60B8, pTmpData, sizeof(Touch_probe_function_00x60B8));
            pTmpData += sizeof(Touch_probe_function_00x60B8);

            memcpy(&Target_velocity_00x60FF, pTmpData, sizeof(Target_velocity_00x60FF));
            pTmpData += sizeof(Target_velocity_00x60FF);

            break;
        case 0x1603:
            memcpy(&Controlword_00x6040, pTmpData, sizeof(Controlword_00x6040));
            pTmpData += sizeof(Controlword_00x6040);

            memcpy(&Modes_of_operation_00x6060, pTmpData, sizeof(Modes_of_operation_00x6060));
            pTmpData += sizeof(Modes_of_operation_00x6060);

            memcpy(&Target_torque_00x6071, pTmpData, sizeof(Target_torque_00x6071));
            pTmpData += sizeof(Target_torque_00x6071);

            memcpy(&Max_torque_00x6072, pTmpData, sizeof(Max_torque_00x6072));
            pTmpData += sizeof(Max_torque_00x6072);

            memcpy(&Target_position_00x607A, pTmpData, sizeof(Target_position_00x607A));
            pTmpData += sizeof(Target_position_00x607A);

            memcpy(&Max_motor_speed_00x6080, pTmpData, sizeof(Max_motor_speed_00x6080));
            pTmpData += sizeof(Max_motor_speed_00x6080);

            memcpy(&Touch_probe_function_00x60B8, pTmpData, sizeof(Touch_probe_function_00x60B8));
            pTmpData += sizeof(Touch_probe_function_00x60B8);

            memcpy(&Target_velocity_00x60FF, pTmpData, sizeof(Target_velocity_00x60FF));
            pTmpData += sizeof(Target_velocity_00x60FF);

            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{
#if _WIN32
   #pragma message ("Warning: Implement the slave application")
#else
    #warning "Implement the slave application"
#endif
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
#if _WIN32
   #pragma message ("Warning: Implement explicit Device ID latching")
#else
    #warning "Implement explicit Device ID latching"
#endif
    /* Explicit Device 5 is expected by Explicit Device ID conformance tests*/
    return 0x5;
}
#endif



#if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
#if _PIC24
int main(void)
#else
void main(void)
#endif
{
    /* initialize the Hardware and the EtherCAT Slave Controller */
#if FC1100_HW
    if(HW_Init())
    {
        HW_Release();
        return;
    }
#else
    HW_Init();
#endif
    MainInit();

    bRunApplication = TRUE;
    do
    {
        MainLoop();
        
    } while (bRunApplication == TRUE);

    HW_Release();
#if _PIC24
    return 0;
#endif
}
#endif //#if USE_DEFAULT_MAIN
/** @} */


