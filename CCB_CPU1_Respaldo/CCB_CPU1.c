//
// Included Files
//

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "F2837xD_Ipc_drivers.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "Monitoreo.h"
#include <math.h>
#include <stdbool.h>
#include "Bit_Control.h"
#include "params.h"
#include "transforms.h"
#include "svm.h"
#include "Faults.h"
#include <CleverControl.h>
#include "PQtheory.h"
#include <CCB_control_mods.h>
#include "CAN_Messages.h"
#include "buffer_circ.h"


#if (CAN_BUS_BASE == CANA_BASE)
#define CAN_USED 1
#define PIN_CAN_RX  GPIO_PIN_CANRXA
#define PIN_CAN_TX  GPIO_PIN_CANTXA
#elif (CAN_BUS_BASE == CANB_BASE)
#define CAN_USED 2
#define PIN_CAN_RX  GPIO_PIN_CANRXB
#define PIN_CAN_TX  GPIO_PIN_CANTXB
#endif

/******************************************************
 ******************** FUNCTIONS ***********************
 ******************************************************/
void initADC(void);
void initEPWM(void);
void initEPWMWithDB(uint32_t base);
void initADCSOC(void);
void ADC1ISR(void);
void ADC2ISR(void);
void Board_init();
void CAN_init();
void EPWM_init();
void PinMux_init();
void configureDAC(void);

void CANsend(uint32_t base, uint32_t objID, uint16_t msgLen, const uint16_t *msgData);
void CANsendStruct(ObjectsCAN *CANstruct, const uint16_t *msgData);
void CanFlagEvaluator();
void CAN_CriticalOrder();
void CAN_Instruction();
void CAN_Request();
void Change_State(bool request);
void Change_Fault(bool request);
void Clear_All_Vectors();
void Check_Time_Request_CAN();
void Clear_Request_CAN_Timer();

void initCPUTimers(void);
void configCPUTimer(uint32_t cpuTimer, float freq, float period);

void readDataCPU1(void);
void writeDataCPU1(void);
/******************************************************
 ******************** INTERRUPTS **********************
 ******************************************************/
__interrupt void cpuTimer0ISR(void);
__interrupt void ePWMISR(void);
__interrupt void ADC3ISR(void);
__interrupt void canISR(void);

void initCPUTimers(void);
void configCPUTimer(uint32_t cpuTimer, float freq, float period);
__interrupt void cpuTimer0ISR(void);

/******************************************************
 ******************** DEFINES  ************************
 ******************************************************/
#define gMSG_DATA_LENGTH        8
#define RESULTS_BUFFER_SIZE     256
#define LENGTH_ARRAY_IPC 9

/******************************************************
 *************** GLOBAL VARIABLES *********************
 ******************************************************/
// IPC VARIABLES //
uint16_t cpu1RArray[gMSG_DATA_LENGTH + 1];       // Mapped to GS0 of shared RAM owned by CPU2
uint16_t cpu1RWArray[gMSG_DATA_LENGTH + 1];      // Mapped to GS1 of shared RAM owned by CPU1
uint16_t sendToCpu2[gMSG_DATA_LENGTH];
uint16_t rcvToCpu2[gMSG_DATA_LENGTH];
#pragma DATA_SECTION(cpu1RArray,"SHARERAMGS2");
#pragma DATA_SECTION(cpu1RWArray,"SHARERAMGS1");
extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncLoadSize;
bool flagRCV = false;
bool flagSND = false;

// CAN VARIABLES //
uint16_t gRxMsgData[gMSG_DATA_LENGTH];
uint16_t gTxMsgData[gMSG_DATA_LENGTH];
uint16_t gFLAGS_CAN_MSG =   0; // Information vector

// ADC VARIABLES //
uint32_t adcResults[RESULTS_BUFFER_SIZE];    // Buffer for results
uint32_t index;                              // Index into result buffer
volatile uint32_t bufferFull;                // Flag to indicate buffer is full
double gVACcalibration[3] = {VAC_BASE, VAC_BASE, VAC_BASE};
double gIACcalibration[3] = {IAC_BASE, IAC_BASE, IAC_BASE};

// FSM VARIABLES //
uint16_t gFLAGS_AND_COMMANDS = 0;
uint16_t gFAULT_VECTOR = 0;
uint16_t LAST_FAULT_VECTOR = 0;
uint16_t gFSM_OUTPUTS = 0;
uint16_t gFSM_STATE = STT_OFF;
uint16_t FSM_LAST_STATE = 0;
uint16_t gWRNG_VECTOR = 0;
uint16_t LAST_WRNG_VECTOR = 0;

// TIMER VAR //
#define FREQUENCY_CAN       300
#define MAX_FREQ_MSG_FAULT  3
#define MAX_WRNG_MSG_FAULT  2
#define PERCENT_FAULT       20
uint8_t frequencyFaultMsg = 0;
uint8_t warningFaultCAN = 0;
bool checkAvailable = false;
unsigned long millisTime = 0;
unsigned long lastMillis = 0;
unsigned long lastMillis_OK = 0;

// OTHERS VARIABLES //
uint16_t gVabc_adc[3] = {2047, 2047, 2047};
uint16_t gIabc_adc[3] = {2047, 2047, 2047};
uint16_t gVdc_adc = 2500;

double gVdc;

//  POWER MESSUREMENT //
double gPseqp;
double gQseqp; // -50KW a 50KW  +-10%
double gPref = 0;
double gQref = 0;

double MAX_Vabc=MAX_AC_VOLTAGE;

uint16_t auxMed1 = 0;
uint16_t auxMed2 = 1;


/******************************************************
 ******************* STRUCTS **************************
 ******************************************************/
struct Overlord gMainOverlord;
struct GA_results gGridAnalysis;
struct StateVariableMonitor gStateMonitor;


ObjectsCAN CAN_RX;
ObjectsCAN CAN_TX;

CIRCULAR_BUFFER_DECLARE(CANbuffer);
CIRCULAR_BUFFER_DECLARE(CAN_SEND_buffer);

/******************************************************
 ********************* MAIN ***************************
 ******************************************************/
void main(void)
{
    Device_init();
    CPUTimer_startTimer(CPUTIMER0_BASE);
    Device_initGPIO();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_register(INT_EPWM1, &ePWMISR);  // Registers a function to be called when an interrupt occurs.
    Interrupt_register(INT_ADCC3, &ADC3ISR);  // Registers a function to be called when an interrupt occurs.

    initCPUTimers();
    initADC();
    initEPWM();
    initADCSOC();
    Board_init();

// Configure the DAC module
    configureDAC();

    initOverlord(&gMainOverlord);

// disable sync and clock to PWM
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);

// Enable epwm interrupt and adc interrupt
    Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_ADCC3);

// enable groups of interrupts
    Interrupt_enable(INTERRUPT_ACK_GROUP1);
    Interrupt_enable(INTERRUPT_ACK_GROUP3);
    Interrupt_enable(INTERRUPT_ACK_GROUP10);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM, 2, SYSCTL_CPUSEL_CPU1);

// enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

// Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

//// Start ePWM1, enabling SOCA and putting the counter in up-down count mode
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

//    Board_init();

    GPIO_setMasterCore(PIN_CAN_RX, GPIO_CORE_CPU2);
    GPIO_setMasterCore(PIN_CAN_TX, GPIO_CORE_CPU2);

    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL8_CAN, CAN_USED, SYSCTL_CPUSEL_CPU2);
    Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

// Give memory access to GS2 and GS14 RAM to CPU2
    MemCfg_setGSRAMMasterSel((MEMCFG_SECT_GS2 | MEMCFG_SECT_GS14),
                             MEMCFG_GSRAMMASTER_CPU2);

// Copy the ISR to a specified RAM location
    memcpy(&isrfuncRunStart, &isrfuncLoadStart, (uint32_t)&isrfuncLoadSize);

    IPCLtoRFlagSet(IPC_FLAG11);

    while(1)
    {
        // RECEIVED FROM CPU 2 //
        flagRCV = IPCRtoLFlagBusy(IPC_FLAG9);
        if(flagRCV == 1)
        {
            GPIO_togglePin(15);
            readDataCPU1();
            CanFlagEvaluator();
            IPCRtoLFlagAcknowledge(IPC_FLAG9);
        }

//        // SEND TO CPU 2 //
        flagSND = IPCLtoRFlagBusy(IPC_FLAG10);
        if(flagSND == 0 && !buffer_empty(&CAN_SEND_buffer))
        {
            writeDataCPU1();
            IPCLtoRFlagSet(IPC_FLAG10);
        }
    }
}

//
// Configure DAC - Setup the reference voltage and output value for the DAC
//
void
configureDAC(void)
{
    //
    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    //
    //DAC_setReferenceVoltage(DACA_BASE, DAC_REF_VDAC);
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);

    //
    // Enable the DAC output
    //
    DAC_enableOutput(DACA_BASE);

    //
    // Set the DAC shadow output to 0
    //
    DAC_setShadowValue(DACA_BASE, 0);

    //
    // Delay for buffered DAC to power up
    //
    DEVICE_DELAY_US(10);

    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    DAC_enableOutput(DACB_BASE);
    DAC_setShadowValue(DACB_BASE, 0);
    DEVICE_DELAY_US(10);
}

void CANsend(uint32_t base, uint32_t objID, uint16_t msgLen, const uint16_t *msgData)
{
//    bool sendFault = true;
//    uint32_t countMsgCanTry = 0;
//    CAN_sendMessage(base, objID, msgLen, msgData);
//    while((((HWREGH(CAN_COMUNIC_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK) && sendFault)
//    {
//        sendFault = MessageCanFail(&gFAULT_VECTOR, &gFLAGS_AND_COMMANDS, countMsgCanTry);
//        countMsgCanTry++;
//    }
}

void CANsendStruct(ObjectsCAN *CANstruct, const uint16_t *msgData)
{
//    bool sendFault = true;
//    uint32_t countMsgCanTry = 0;
//    CAN_sendMessage(CANstruct->base, CANstruct->mailBox, CANstruct->msgLen, msgData);
    uint16_t sendData[gMSG_DATA_LENGTH];
    int i; for(i = 0; i < gMSG_DATA_LENGTH; i++)
    {
        sendData[i] = msgData[i];
    }
    buffer_insert(&CAN_SEND_buffer, &sendData, gMSG_DATA_LENGTH);
    uint16_t idSend = CANstruct->ID;
    buffer_insert(&CAN_SEND_buffer, &idSend, 1);
//    while((((HWREGH(CANstruct->base + CAN_O_ES) & CAN_ES_TXOK)) != CAN_ES_TXOK) && sendFault)
//    {
//        sendFault = MessageCanFail(&gFAULT_VECTOR, &gFLAGS_AND_COMMANDS, countMsgCanTry);
//        countMsgCanTry++;
//    }
}

__interrupt void canISR(void)
{
//    GPIO_writePin(GPI4, true);
//    uint16_t status = CAN_getInterruptCause(CAN_COMUNIC_BASE);
//    uint16_t receivedDataCAN[gMSG_DATA_LENGTH];
//    uint16_t flagToSet = 0;
//    CANread(&CAN_RX, receivedDataCAN);
//
//    if (status == CAN_RX.mailBox)
//    {
//        bool  receivedIsValid = false;
//        switch (CAN_RX.lastRxID)
//        {
//            case ID_CRITICAL_ORDER:
//                CANread(&CAN_RX, receivedDataCAN);
//                switch (receivedDataCAN[B_DIR])
//                {
//                    case DIRB_RX_EMERGENCY_STOP:
//                        EmergencyStop(&gFAULT_VECTOR, &gFLAGS_AND_COMMANDS); break;
//                }
//                buffer_flush(&CANbuffer);
//                break;
//            case ID_INSTRUCTION:
//                receivedIsValid = true; flagToSet = gFlagInstruction;
//                break;
//            case ID_REQUEST:
//                receivedIsValid = true; flagToSet = gFlagRequest;
//                Clear_Request_CAN_Timer();
//                break;
//            default:
//                break;
//        }
//        if (receivedIsValid)
//        {
//            buffer_insert(&CANbuffer, receivedDataCAN, gMSG_DATA_LENGTH);
//            buffer_insert(&CANbuffer, &flagToSet, 1);
//        }
//
//        CAN_clearInterruptStatus(CAN_RX.base, CAN_RX.mailBox);
//    }
//
//    CAN_clearGlobalInterruptStatus(CAN_COMUNIC_BASE, CAN_GLOBAL_INT_CANINT0);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
//
//    GPIO_writePin(GPI4, false);
}

void CanFlagEvaluator()
{
    if(Bit_Read(gFLAGS_CAN_MSG, gFlagCriticalOrder))
    {
        CAN_CriticalOrder();
    }
    if(Bit_Read(gFLAGS_CAN_MSG, gFlagInstruction))
    {
        CAN_Instruction();
    }
    if (Bit_Read(gFLAGS_CAN_MSG, gFlagRequest))
    {
        CAN_Request();
    }
    gFLAGS_CAN_MSG = 0;
}

void CAN_CriticalOrder()
{
    switch (gRxMsgData[B_DIR])
    {
    case DIR_RX_EMERGENCY_STOP:
        EmergencyStop(&gFAULT_VECTOR, &gFLAGS_AND_COMMANDS);
        break;
    default: break;
    }
}

void CAN_Instruction()
{
    switch (gRxMsgData[B_DIR])
    {
    case DIR_RX_PRELOAD:
        (gRxMsgData[1]) ?   Bit_Write(&gFLAGS_AND_COMMANDS, CMD_PREDC_START, true)
                       :    Bit_Write(&gFLAGS_AND_COMMANDS, CMD_PREDC_STOP, true);
        break;
    case DIR_RX_CTTDC:
        (gRxMsgData[1]) ?   Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTTDC_MAKE, true)
                       :    Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTTDC_BREAK, true);
        break;
    case DIR_RX_CTTAC:
        (gRxMsgData[1]) ?   Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTTAC_MAKE, true)
                       :    Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTTAC_BREAK, true);
        break;
    case DIR_RX_RESET:
        Clear_All_Vectors(); Bit_Write(&gFLAGS_AND_COMMANDS, CMD_RESET, true);
        buffer_flush(&CANbuffer); //Limpia la cola del buffer
        break;
    case DIR_RX_SET_PWR:
        gPref = (double)hexToInt_3(gRxMsgData[1], gRxMsgData[2], gRxMsgData[3]) - 90000.0;
        gQref = (double)hexToInt_3(gRxMsgData[4], gRxMsgData[5], gRxMsgData[6]) - 90000.0;
//        gStateMonitor.PQ_seqp_ref[0] = (double)hexToInt_3(gRxMsgData[1], gRxMsgData[2], gRxMsgData[3]) - 50000.0;
//        gStateMonitor.PQ_seqp_ref[1] = (double)hexToInt_3(gRxMsgData[4], gRxMsgData[5], gRxMsgData[6]) - 50000.0;
        break;
    case 0xA8:
        Bit_Write(&gFLAGS_AND_COMMANDS, FLG_PREDCOK, gRxMsgData[1]); break;
    case 0xA9:
        Bit_Write(&gFLAGS_AND_COMMANDS, FLG_FAULT, gRxMsgData[1]); break;
    case 0xAA:
        Bit_Write(&gFLAGS_AND_COMMANDS, FLG_PLLOK, gRxMsgData[1]); break;
    case 0xA4:
        (gRxMsgData[1]) ?    Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTRL_ENABLE, true)
                        :    Bit_Write(&gFLAGS_AND_COMMANDS, CMD_CTRL_DISABLE, true);
        break;
    default: break;
    }

}

void CAN_Request()
{
    uint16_t intValue_16 = 0;
    uint32_t intValue_32 = 0;
    switch (gRxMsgData[B_DIR])
    {
    case DIR_RX_CURRENT_STATE:
        Change_State(STATUS_CURRENT_STATE); break;
    case 0xA0:
        CAN_TX.ID = ID_REPLY;
        CANsendStruct(&CAN_TX, gRxMsgData);
        break;
    case DIR_RX_VOLT_FREQ_GRID:
        gTxMsgData[B_DIR] = REPLY_VOLT_FREQ;
        intValue_16 = (uint16_t)((gGridAnalysis.Vfnrms_seqp / 260.0) * (DISPLACEMENT_15 - 1));
        IntToHex_2(intValue_16, &gTxMsgData[1], &gTxMsgData[2]);
        intValue_16 = 0;
        intValue_16 = (uint16_t)(((gGridAnalysis.omega/(2*PI) - 50.0)/2.0) * DISPLACEMENT_11 + DISPLACEMENT_11);
        IntToHex_2(intValue_16, &gTxMsgData[3], &gTxMsgData[4]);
        CAN_TX.ID = ID_REPLY;
        CANsendStruct(&CAN_TX, gTxMsgData);
        break;
    case DIR_RX_LEVEL_P_Q:
        gTxMsgData[B_DIR] = REPLY_LEVEL_P_Q;
        intValue_32 = (((gPseqp + 90000) / 180000) * (DISPLACEMENT_24 - 1));
//        intValue_32 = (((gStateMonitor->PQp0_seqp_inst[0] + 55000) / 110000) * (DISPLACEMENT_24 - 1));
        IntToHex_3(intValue_32, &gTxMsgData[1], &gTxMsgData[2], &gTxMsgData[3]);
        intValue_32 = 0;
        intValue_32 = (((gQseqp + 90000) / 180000) * (DISPLACEMENT_24 - 1));
//        intValue_32 = (((gStateMonitor->PQp0_seqp_inst[1] + 55000) / 110000) * (DISPLACEMENT_24 - 1));
        IntToHex_3(intValue_32, &gTxMsgData[4], &gTxMsgData[5], &gTxMsgData[6]);
        CAN_TX.ID = ID_REPLY;
        CANsendStruct(&CAN_TX, gTxMsgData);
        break;
    case DIR_RX_VOLT_DC:
        gTxMsgData[B_DIR] = REPLY_VOLT_DC;
        intValue_16 = ((gVdc / 1000) * (DISPLACEMENT_12 - 1));
//        intValue_16 = ((gStateMonitor->Vdc_filtered / 1000) * (DISPLACEMENT_12 - 1));
        IntToHex_2(intValue_16, &gTxMsgData[1], &gTxMsgData[2]);
        CAN_TX.ID = ID_REPLY;
        CANsendStruct(&CAN_TX, gTxMsgData);
        break;
    case DIR_RX_FAULT_VECTOR:
        Change_Fault(STATUS_CURRENT_STATE);
        break;
    default : break;
    }
}

void Change_State(bool request)
{
    if (gFSM_STATE != FSM_LAST_STATE || request)
    {
        gTxMsgData[B_DIR] = 0xA1;
        gTxMsgData[1] = gFSM_STATE;

        (request) ? (CAN_TX.ID = ID_REPLY):
                (CAN_TX.ID = ID_INFORMATION);

        CANsendStruct(&CAN_TX, gTxMsgData);

        FSM_LAST_STATE = gFSM_STATE;
    }
}

void Change_Fault(bool request)
{
    if (gFAULT_VECTOR != LAST_FAULT_VECTOR || request)
    {
        IntToHex_2(gFAULT_VECTOR, &gTxMsgData[1], &gTxMsgData[2]);
        if (request)
        {
            gTxMsgData[B_DIR] = REPLY_FAIL_STATE;
            CAN_TX.ID = ID_REPLY;
        }
        else
        {
            gTxMsgData[B_DIR] = FAIL_MSG;
            CAN_TX.ID = ID_FAIL_MSG;
        }
        CANsendStruct(&CAN_TX, gTxMsgData);
        LAST_FAULT_VECTOR = gFAULT_VECTOR;
    }
}

void Clear_All_Vectors()
{
    checkAvailable = false;
    gWRNG_VECTOR = 0;
    LAST_WRNG_VECTOR = 0;
    warningFaultCAN = 0;
    frequencyFaultMsg = 0;
}

void Clear_Request_CAN_Timer()
{
    lastMillis = millisTime;
    lastMillis_OK = millisTime;
    warningFaultCAN = 0;
    frequencyFaultMsg = 0;
    checkAvailable = false;
}

void Check_Time_Request_CAN()
{
    bool failRequest = false;
    if (gFSM_STATE == STT_DCREADY || gFSM_STATE == STT_STANDBY || gFSM_STATE == STT_ON)
    {
        checkAvailable = true;
    }
    else
    {
        Clear_Request_CAN_Timer();
        return;
    }
    if (millisTime - lastMillis >= FREQUENCY_CAN && checkAvailable)
    {
        lastMillis = millisTime;
        if (frequencyFaultMsg == MAX_FREQ_MSG_FAULT)
        {
            frequencyFaultMsg = 0;
            failRequest = true;
            if (warningFaultCAN < MAX_FREQ_MSG_FAULT)
            {
                ClearArray(gTxMsgData, gMSG_DATA_LENGTH);
                gTxMsgData[B_DIR] = WARNING_MSG;
                IntToHex_2(gWRNG_VECTOR, &gTxMsgData[1], &gTxMsgData[2]);
                CAN_TX.ID = ID_FAIL_MSG;
                CANsendStruct(&CAN_TX, gTxMsgData);
                lastMillis_OK = millisTime;
            }
        }
        else
        {

        }
        if (failRequest) warningFaultCAN++;
        frequencyFaultMsg++;
    }

    if (warningFaultCAN == MAX_WRNG_MSG_FAULT && millisTime - lastMillis_OK >= FREQUENCY_CAN * (PERCENT_FAULT / 100.0))
    {
        Bit_Write(&gFAULT_VECTOR, FLT_TIMEOUT_CAN, true);
        Bit_Write(&gFLAGS_AND_COMMANDS, FLG_FAULT, true);
    }
}

void initADC(void)
{
    // Set ADCCLK divider to /1
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);

    // Set resolution and signal mode

    // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz SYSCLK rate) will be used.
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    // Set pulse positions to late
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    // Power up the ADC and then delay for 1 ms
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);

    XBAR_setInputPin(XBAR_INPUT1, GPI1);
}

void initEPWM(void)
{
//***********************************Configuration epwm 1a y 1b********************************************
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN); // counter in UP_DOWM mode
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    // Set the compare A value to 4990 and the period to 5000
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0x137e);
    EPWM_setTimeBasePeriod(EPWM1_BASE, PRD_PWM_TRIGGER);
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    // enabling interrupt of pwm1 and interrupt source
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_PERIOD);
    // Sets the ePWM interrupt event counts.
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);
    // ADC trigger configuration
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1U);


//************************************+ Configuration epwm 2a y 2b********************************************
    initEPWMWithDB(EPWM2_BASE);

//*****************************configuration epwm 5a y 5b*************************************
    initEPWMWithDB(EPWM5_BASE);


//************************************************* Configuration epwm 6a y 6b********************************************************+
    initEPWMWithDB(EPWM6_BASE);

}

void initEPWMWithDB(uint32_t base)
{
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDelayCount(base, 400);
    EPWM_setRisingEdgeDelayCount(base, 400);

    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    // Use the delayed signals instead of the original signals
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    // DO NOT Switch Output A with Output B
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);

    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1); //Set the time base clock and the high speed time base clock count pre-scaler
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setTimeBasePeriod(base, PRD_PWM_OUT);  // Set the period value to 4095
//    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
//    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
//    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_OSHT1);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCAEVT1);
    EPWM_selectDigitalCompareTripInput(base, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_DCAEVT1);

}

void initADCSOC(void)
{
    // Configure SOC0 of ADCA to convert pin A3. The EPWM1SOCA signal will be the trigger.
    // Configure SOC0 of ADCB to convert pin A3. The EPWM1SOCA signal will be the trigger.
    // Configure SOC0 of ADCC to convert pin A3. The EPWM1SOCA signal will be the trigger.

    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN3, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN3, 15);

    // Set SOC0 to set the interrupt 1 flag. make sure its flag is cleared.
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    // Configure SOC1 of ADCA to convert pin A5. The ADCINT1 signal will be the trigger.
    // Configure SOC1 of ADCB to convert pin A5. The ADCINT1 signal will be the trigger.
    // Configure SOC1 of ADCC to convert pin A5. The ADCINT1 signal will be the trigger.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 15);

    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);

    // Set SOC1 to set the interrupt 2 flag. make sure its flag is cleared.
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);

    // Configure SOC2 of ADCC to convert pin C2 y C4. The ADCINT2 signal will be the trigger.
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 15);

    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_ADCINT2);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_ADCINT2);

    // Set SOC2 to set the interrupt 3 flag. make sure its flag is cleared.
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER3, ADC_SOC_NUMBER3);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER3);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER3);

}

__interrupt void ePWMISR(void)
{
    // Add the latest result to the buffer
    //adcResults[index++] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    //adcResults[index++] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    //adcResults[index++] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);

    // Current sensors
    // Ia = C3
    // Ib = B3
    // Ic = A3
    gIabc_adc[0] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    gIabc_adc[1] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    gIabc_adc[2] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);


    ADC1ISR();

    ADC2ISR();

    // clear interrupt flags in groups and epwm
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //GPIO_togglePin(GPI4);
}

 void ADC1ISR(void){
    // Add the latest result to the buffer
    //adcResults[index++] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
    //adcResults[index++] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    //adcResults[index++] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);

    // Set the bufferFull flag if the buffer is full
    //if(RESULTS_BUFFER_SIZE <= index)
    // {
    //    index = 0;
    //    bufferFull = 1;
    // }

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
     {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
     }

    if(true == ADC_getInterruptOverflowStatus(ADCB_BASE, ADC_INT_NUMBER1))
     {
        ADC_clearInterruptOverflowStatus(ADCB_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
     }

    if(true == ADC_getInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER1))
     {
        ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
     }

    // Grid voltage sensors
    // Va = C5
    // Vb = B5
    // Vc = A5

    gVabc_adc[0] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
    gVabc_adc[1] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    gVabc_adc[2] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);

    // clear interrupt flags
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
}

 void ADC2ISR(void){

    // Add the latest result to the buffer
    //adcResults[index++] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);
    //adcResults[index++] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);

    // Set the bufferFull flag if the buffer is full
    //if(RESULTS_BUFFER_SIZE <= index)
    // {
    //    index = 0;
    //    bufferFull = 1;
    // }

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER2);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);
    }

    // Reference Voltage = C2
//    gVref_adc = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);
    //gVref_adc = 1861;

    // DC voltage = C4
    gVdc_adc = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    //gVdc_adc = 372;

    // clear interrupt flags
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER3);
}


 __interrupt void ADC3ISR(void)
 {
    // Set the bufferFull flag if the buffer is full
    //if(RESULTS_BUFFER_SIZE <= index)
    // {
    //    index = 0;
    //    bufferFull = 1;
    // }

    // Check if overflow has occurred
    if(true == ADC_getInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER3))
     {
        ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER3);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER3);
     }

    // parameter setting for variable calculations

    // clear interrupt flags
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER3);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

//    Check_Time_Request_CAN();
    Change_State(STATUS_CHECK_CHANGE);
    Change_Fault(STATUS_CHECK_CHANGE);


//    stepOverlord(&gMainOverlord,
//                 &gFLAGS_AND_COMMANDS, &gFSM_OUTPUTS, &gFAULT_VECTOR, &gFSM_STATE,
//                 gVabc_adc, gIabc_adc, gVdc_adc,
//                 gVACcalibration, gIACcalibration,
//                 gPref, gQref,
//                 &gPseqp, &gQseqp, &gVdc,
//                 &gGridAnalysis);

    stepOverlord(&gMainOverlord,
                 &gFLAGS_AND_COMMANDS, &gFSM_OUTPUTS, &gFAULT_VECTOR, &gFSM_STATE,
                 gVabc_adc, gIabc_adc, gVdc_adc,
                 gVACcalibration, gIACcalibration,
                 gPref, gQref,
                 &gPseqp, &gQseqp, &gVdc,
                 &gGridAnalysis,
                 &gStateMonitor);

    double radius = 0.5;
    double freq = gGridAnalysis.omega / _2PI;
    freq -= 50.0;
    freq /= 2.0*radius;
    freq += 0.5;
    freq = MAX(0.0, MIN(freq, 1.0));
    freq *= 4095;

    DAC_setShadowValue(DACB_BASE, freq);
    DAC_setShadowValue(DACA_BASE, 0.0);

    ClearArray(gTxMsgData, gMSG_DATA_LENGTH);
    ClearArray(gTxMsgData, gMSG_DATA_LENGTH);
}

 //
 // initCPUTimers - This function initializes all three CPU timers
 // to a known state.
 //
 void initCPUTimers(void)
 {
     //
     // Initialize timer period to maximum
     //
     CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);

     //
     // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
     //
     CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

     //
     // Make sure timer is stopped
     //
     CPUTimer_stopTimer(CPUTIMER0_BASE);

     //
     // Reload all counter register with period value
     //
     CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

     //
     // Reset interrupt counter
     //
 }

 //
 // configCPUTimer - This function initializes the selected timer to the
 // period specified by the "freq" and "period" parameters. The "freq" is
 // entered as Hz and the period in uSeconds. The timer is held in the stopped
 // state after configuration.
 //
 void configCPUTimer(uint32_t cpuTimer, float freq, float period)
 {
     uint32_t temp;

     //
     // Initialize timer period:
     //
     temp = (uint32_t)(freq / 1000000 * period);
     CPUTimer_setPeriod(cpuTimer, temp);

     //
     // Set pre-scale counter to divide by 1 (SYSCLKOUT):
     //
     CPUTimer_setPreScaler(cpuTimer, 0);

     //
     // Initializes timer control register. The timer is stopped, reloaded,
     // free run disabled, and interrupt enabled.
     // Additionally, the free and soft bits are set
     //
     CPUTimer_stopTimer(cpuTimer);
     CPUTimer_reloadTimerCounter(cpuTimer);
     CPUTimer_setEmulationMode(cpuTimer,
                            CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
     CPUTimer_enableInterrupt(cpuTimer);
 }

 __interrupt void cpuTimer0ISR(void)
 {
     millisTime++;
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
 }

//
// writeDataCPU1 - Write a pattern to an array in shared RAM
//
void writeDataCPU1(void)
{
    buffer_retrieve(&CAN_SEND_buffer, sendToCpu2, gMSG_DATA_LENGTH + 1);
    uint16_t index;
    for(index = 0; index < gMSG_DATA_LENGTH + 1; index++)
    {
        cpu1RWArray[index] = sendToCpu2[index];
    }
}

//
// readDataCPU1 - Read and compare an array from shared RAM
//
void readDataCPU1(void)
{
    uint16_t index;
    for(index = 0; index < gMSG_DATA_LENGTH; index++)
    {
        rcvToCpu2[index] = cpu1RArray[index];
        gRxMsgData[index] = rcvToCpu2[index];
    }
    Bit_Write(&gFLAGS_CAN_MSG, cpu1RArray[gMSG_DATA_LENGTH], 1);
    CanFlagEvaluator();

}

//
// End of File
//
