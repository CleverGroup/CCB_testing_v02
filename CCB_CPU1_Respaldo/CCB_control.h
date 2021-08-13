#include "FiniteStateMachine.h"


// Number of Inputs
#define NumInputs 8

// State Definitions
#define STT_OFF      1
#define STT_FAULT    2
#define STT_PRELOAD  3
#define STT_DCREADY  4
#define STT_SYNCHING 5
#define STT_STANDBY  6
#define STT_ON       7


// Definition of output bits
#define OUT_ENCTR 0
#define OUT_PREDC 1
#define OUT_CTTDC 2
#define OUT_CTTAC_sig 3
#define OUT_CTTAC_pwr 4
#define OUT_FAN   5


// Definition of input bits
// Internal flags (FLG)
#define FLG_FAULT   0
#define FLG_PLLOK   1
#define FLG_PREDCOK 2
// External commands (CMD)
#define CMD_CTRL_ENABLE  3
#define CMD_CTRL_DISABLE 4
#define CMD_CTTAC_MAKE   5
#define CMD_CTTAC_BREAK  6
#define CMD_CTTDC_MAKE   7
#define CMD_CTTDC_BREAK  8
#define CMD_PREDC_START  9
#define CMD_PREDC_STOP   10
#define CMD_RESET        11




////////////////////////////////////////////////////////////////////////////////////////////////
// PARTICULAR CODE FOR THE MAIN FSM IMPLEMENTATION

void initCCBFSM(struct FSM *pFSMvar){

    pFSMvar->initialState = STT_OFF;
    pFSMvar->currentState = STT_OFF;

    pFSMvar->initialOutput = 0;
    pFSMvar->currentOutput = 0;

////////////////////////////////////////////////////////////////////////////////////////////
//  State Definitions
////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////
    //  State NULL
    ///////////////////////
    pFSMvar->stateSet[0].name = 0;
    pFSMvar->stateSet[0].outputVector = 0;
    pFSMvar->stateSet[0].numOutTrans = 0;

    ///////////////////////
    //  State OFF
    ///////////////////////
    pFSMvar->stateSet[STT_OFF].name = STT_OFF;
    pFSMvar->stateSet[STT_OFF].outputVector = (0 << OUT_CTTDC) | (0 << OUT_CTTAC_sig) | (0 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_OFF].numOutTrans = 2;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_OFF].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_OFF].negateInput[0] = false;
    pFSMvar->stateSet[STT_OFF].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_OFF].inputMasks[1] = (1 << CMD_PREDC_START);
    pFSMvar->stateSet[STT_OFF].negateInput[1] = false;
    pFSMvar->stateSet[STT_OFF].nextState[1] = STT_PRELOAD;


    ///////////////////////
    //  State FAULT
    ///////////////////////
    pFSMvar->stateSet[STT_FAULT].name = STT_FAULT;
    pFSMvar->stateSet[STT_FAULT].outputVector = (0 << OUT_CTTDC) | (0 << OUT_CTTAC_sig) | (0 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_FAULT].numOutTrans = 1;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_FAULT].inputMasks[0] = (1 << CMD_RESET);
    pFSMvar->stateSet[STT_FAULT].negateInput[0] = false;
    pFSMvar->stateSet[STT_FAULT].nextState[0] = STT_OFF;


    ///////////////////////
    //  State PRELOAD
    ///////////////////////
    pFSMvar->stateSet[STT_PRELOAD].name = STT_PRELOAD;
    pFSMvar->stateSet[STT_PRELOAD].outputVector = (0 << OUT_CTTDC) | (0 << OUT_CTTAC_sig) | (0 << OUT_CTTAC_pwr) | (1 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_PRELOAD].numOutTrans = 3;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_PRELOAD].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_PRELOAD].negateInput[0] = false;
    pFSMvar->stateSet[STT_PRELOAD].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_PRELOAD].inputMasks[1] = (1 << CMD_PREDC_STOP);
    pFSMvar->stateSet[STT_PRELOAD].negateInput[1] = false;
    pFSMvar->stateSet[STT_PRELOAD].nextState[1] = STT_OFF;
    // T2
    pFSMvar->stateSet[STT_PRELOAD].inputMasks[2] = (1 << FLG_PREDCOK);
    pFSMvar->stateSet[STT_PRELOAD].negateInput[2] = false;
    pFSMvar->stateSet[STT_PRELOAD].nextState[2] = STT_DCREADY;


    ///////////////////////
    //  State DCREADY
    ///////////////////////
    pFSMvar->stateSet[STT_DCREADY].name = STT_DCREADY;
    pFSMvar->stateSet[STT_DCREADY].outputVector = (1 << OUT_CTTDC) | (0 << OUT_CTTAC_sig) | (0 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_DCREADY].numOutTrans = 4;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_DCREADY].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_DCREADY].negateInput[0] = false;
    pFSMvar->stateSet[STT_DCREADY].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_DCREADY].inputMasks[1] = (1 << CMD_CTTDC_BREAK);
    pFSMvar->stateSet[STT_DCREADY].negateInput[1] = false;
    pFSMvar->stateSet[STT_DCREADY].nextState[1] = STT_OFF;
    // T2
    pFSMvar->stateSet[STT_DCREADY].inputMasks[2] = (1 << FLG_PREDCOK);
    pFSMvar->stateSet[STT_DCREADY].negateInput[2] |= (1 << FLG_PREDCOK);
    pFSMvar->stateSet[STT_DCREADY].nextState[2] = STT_PRELOAD;
    // T3
    pFSMvar->stateSet[STT_DCREADY].inputMasks[3] = (1 << CMD_CTTAC_MAKE);
    pFSMvar->stateSet[STT_DCREADY].negateInput[3] = false;
    pFSMvar->stateSet[STT_DCREADY].nextState[3] = STT_SYNCHING;


    ///////////////////////
    //  State SYNCHING
    ///////////////////////
    pFSMvar->stateSet[STT_SYNCHING].name = STT_SYNCHING;
    pFSMvar->stateSet[STT_SYNCHING].outputVector = (1 << OUT_CTTDC) | (1 << OUT_CTTAC_sig) | (0 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_SYNCHING].numOutTrans = 3;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_SYNCHING].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_SYNCHING].negateInput[0] = false;
    pFSMvar->stateSet[STT_SYNCHING].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_SYNCHING].inputMasks[1] = (1 << CMD_CTTAC_BREAK);
    pFSMvar->stateSet[STT_SYNCHING].negateInput[1] = false;
    pFSMvar->stateSet[STT_SYNCHING].nextState[1] = STT_DCREADY;
    // T2
    pFSMvar->stateSet[STT_SYNCHING].inputMasks[2] = (1 << FLG_PLLOK);
    pFSMvar->stateSet[STT_SYNCHING].negateInput[2] = false;
    pFSMvar->stateSet[STT_SYNCHING].nextState[2] = STT_STANDBY;


    ///////////////////////
    //  State STANDBY
    ///////////////////////
    pFSMvar->stateSet[STT_STANDBY].name = STT_STANDBY;
    pFSMvar->stateSet[STT_STANDBY].outputVector = (1 << OUT_CTTDC) | (1 << OUT_CTTAC_sig) | (1 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (0 << OUT_ENCTR) | (0 << OUT_FAN);
    pFSMvar->stateSet[STT_STANDBY].numOutTrans = 4;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_STANDBY].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_STANDBY].negateInput[0] = false;
    pFSMvar->stateSet[STT_STANDBY].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_STANDBY].inputMasks[1] = (1 << CMD_CTTAC_BREAK);
    pFSMvar->stateSet[STT_STANDBY].negateInput[1] = false;
    pFSMvar->stateSet[STT_STANDBY].nextState[1] = STT_DCREADY;
    // T2
    pFSMvar->stateSet[STT_STANDBY].inputMasks[2] = (1 << FLG_PLLOK);
    pFSMvar->stateSet[STT_STANDBY].negateInput[2] |= (1 << FLG_PLLOK);
    pFSMvar->stateSet[STT_STANDBY].nextState[2] = STT_SYNCHING;
    // T3
    pFSMvar->stateSet[STT_STANDBY].inputMasks[3] = (1 << CMD_CTRL_ENABLE);
    pFSMvar->stateSet[STT_STANDBY].negateInput[3] = false;
    pFSMvar->stateSet[STT_STANDBY].nextState[3] = STT_ON;


    ///////////////////////
    //  State ON
    ///////////////////////
    pFSMvar->stateSet[STT_ON].name = STT_ON;
    pFSMvar->stateSet[STT_ON].outputVector = (1 << OUT_CTTDC) | (1 << OUT_CTTAC_sig) | (1 << OUT_CTTAC_pwr) | (0 << OUT_PREDC) | (1 << OUT_ENCTR) | (1 << OUT_FAN);
    pFSMvar->stateSet[STT_ON].numOutTrans = 3;
    // Transitions Ordered by priority
    // T0
    pFSMvar->stateSet[STT_ON].inputMasks[0] = (1 << FLG_FAULT);
    pFSMvar->stateSet[STT_ON].negateInput[0] = false;
    pFSMvar->stateSet[STT_ON].nextState[0] = STT_FAULT;
    // T1
    pFSMvar->stateSet[STT_ON].inputMasks[1] = (1 << CMD_CTRL_DISABLE);
    pFSMvar->stateSet[STT_ON].negateInput[1] = false;
    pFSMvar->stateSet[STT_ON].nextState[1] = STT_STANDBY;
    // T2
    pFSMvar->stateSet[STT_ON].inputMasks[2] = (1 << FLG_PLLOK);
    pFSMvar->stateSet[STT_ON].negateInput[2] |= (1 << FLG_PLLOK);
    pFSMvar->stateSet[STT_ON].nextState[2] = STT_SYNCHING;




// STATE PROTOTYPE
/////////////////////////
////  State
/////////////////////////
//    pFSMvar->stateSet[STT_].name = ;
//    pFSMvar->stateSet[STT_].outputVector = ( << OUT_CTTDC) | ( << OUT_CTTAC_sig) | ( << OUT_CTTAC_pwr) | ( << OUT_PREDC) | ( << OUT_ENCTR);
//    pFSMvar->stateSet[STT_].numOutTrans = ;
//    // Transitions Ordered by priority
//    // T0
//    pFSMvar->stateSet[STT_].inputMasks[0] = (1 << );
//    pFSMvar->stateSet[STT_].negateInput[0] |= ;
//    pFSMvar->stateSet[STT_].nextState[0] = ;


}


////////////////////////////////////////////////////////////////////////////////////////////////
// Overlord controller manages everything that must be done when control interruptions occur. Battery connected.

struct Overlord{

    struct FSM mainFSM;
    struct RobustPQctrl RPQCvar;
    struct GridAnalyser GridAnalVar;
    struct PreloadCtrl PreDCvar;

    struct ADC_calibration ADCcalVar;

    uint16_t FSMstate;
    uint16_t prevFSMstate;
    bool onEntry;

};

void initOverlord(struct Overlord *pOverlordVar,
                  double Ts,
                  double kpC, double kiC, double nominalReactance, double beta_d_eq,
                  double kpP, double kiP,
                  double kpQ, double kiQ,
                  double VDCcutoff_freq, double VDCinit,
                  int harmonic1, double kpH1, double krH1,
                  int harmonic2, double kpH2, double krH2,
                  double kpPLL, double kiPLL,
                  double sogiGain, double wg_nom, double vfn_rms_nom,
                  double pll_error_thresh, double pll_error_tc, double pll_error_lpf_init,
                  int Vdc_ref, int preDC_thresh, int minVdc){


    initRPQctrl(&(pOverlordVar->RPQCvar),
                Ts,
                kpC, kiC, nominalReactance, beta_d_eq,
                kpP, kiP, kpQ, kiQ,
                VDCcutoff_freq, VDCinit,
                harmonic1, kpH1, krH1, harmonic2, kpH2, krH2);

    initGridAnalyser(&(pOverlordVar->GridAnalVar),
                   Ts, kpPLL, kiPLL,
                   sogiGain, wg_nom, vfn_rms_nom, pll_error_thresh, pll_error_tc, pll_error_lpf_init);

    initCCBFSM(&(pOverlordVar->mainFSM));

    pOverlordVar->FSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->prevFSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->onEntry = false;

    initPLctrl(&(pOverlordVar->PreDCvar), Vdc_ref, preDC_thresh, minVdc);
    initADCcalibration(&(pOverlordVar->ADCcalVar), Ts, OFFSET_CALIBRATION_LPF_FREQ, OFFSET_CALIBRATION_LPF_INIT);

}

void resetOverlord(struct Overlord *pOverlordVar,
                   uint16_t *pFlagsAndCommands, uint16_t *pOutputVector, uint16_t *pFaultVector){


    resetRPQctrl(&(pOverlordVar->RPQCvar));
    resetFSM(&(pOverlordVar->mainFSM));
    resetPLctrl(&(pOverlordVar->PreDCvar));

    pOverlordVar->FSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->prevFSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->onEntry = false;

//    clear all flags and commands
    clearVector(pFlagsAndCommands);
    clearVector(pOutputVector);
    clearVector(pFaultVector);

    resetADCcalibration(&(pOverlordVar->ADCcalVar), OFFSET_CALIBRATION_LPF_INIT);

}

void turnAllOff(){
    EPWM_forceTripZoneEvent(EPWM2_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
    EPWM_forceTripZoneEvent(EPWM5_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
    EPWM_forceTripZoneEvent(EPWM6_BASE, EPWM_TZ_FORCE_EVENT_DCAEVT1);
}

void turnAllOn(uint32_t base){
    /*
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    */
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_NO_CHANGE,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_NO_CHANGE,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_clearOneShotTripZoneFlag(base,
                                  EPWM_TZ_OST_FLAG_OST1);
    EPWM_clearTripZoneFlag(base,
                           EPWM_TZ_FLAG_OST);
}

void stepOverlord(struct Overlord *pOverlordVar,
                  uint16_t *pFlagsAndCommands, uint16_t *pOutputVector, uint16_t *pFaultVector, uint16_t *pFSMstate,
                  uint16_t *gVabc_adc, uint16_t *gIabc_adc, uint16_t gVdc_adc, double *gVACcalibration,
                  double *gIACcalibration, double gPref, double gQref,
                  double *pgPseqp, double *pgQseqp,
                  double *pgVdc, struct GA_results *pgGridAnalysis){

    // LOCAL VARIABLES //


    bool converged = false;
    struct GA_results GridAnalOutputs;
    bool preloadReady;
    bool FLG_READY_PREDC;
    double dutyCycleABCout[3];


    //    pending: read GPI's and set faults if necesary
    if (GPIO_readPin(GPI_Emergency_Stop))
    {
        EmergencyStop(&*pFaultVector, &*pFlagsAndCommands);
    }


    // Scale ADC readings
    double Vabc[3];
    double Iabc[3];
    double Vdc;
    measurementConversion(gVabc_adc, gIabc_adc, gVdc_adc, gVACcalibration, gIACcalibration,
                          Vabc, Iabc, &Vdc);
    *pgVdc = Vdc;



    // Determine Faults
    if (pOverlordVar->FSMstate > STT_PRELOAD){
        testVdcLimSup(pFaultVector, pFlagsAndCommands, Vdc);
    }
    if (pOverlordVar->FSMstate > STT_DCREADY){
        testVdcLimInf(pFaultVector, pFlagsAndCommands, Vdc); // no hay que revisar límite inferior en estado de precarga
    }
    if (pOverlordVar->FSMstate > STT_SYNCHING){
        testVac(pFaultVector, pFlagsAndCommands, Vabc);
    }
    if (pOverlordVar->FSMstate > STT_STANDBY){
        testNoGrid(pFaultVector, pFlagsAndCommands, (pgGridAnalysis->Vfnrms_seqp));
        testIac(pFaultVector, pFlagsAndCommands, Iabc);
    }




    // Update FSM
    pOverlordVar->prevFSMstate = pOverlordVar->FSMstate;
    pOverlordVar->FSMstate = stepFSM(&(pOverlordVar->mainFSM), *pFlagsAndCommands, pOutputVector);
    pOverlordVar->onEntry = (pOverlordVar->FSMstate != pOverlordVar->prevFSMstate);


    if (*pFSMstate == STT_FAULT && Bit_Read(*pFlagsAndCommands, CMD_RESET))
   {
       resetOverlord(pOverlordVar, pFlagsAndCommands, pOutputVector, pFaultVector);
   }
   *pFSMstate = pOverlordVar->FSMstate;

   if(Bit_Read(*pFlagsAndCommands, CMD_CTTDC_MAKE)) FLG_READY_PREDC = true;
   else FLG_READY_PREDC = false;



   // clear all commands
   Bit_Write(pFlagsAndCommands, CMD_CTRL_DISABLE, false); // 3
   Bit_Write(pFlagsAndCommands, CMD_CTRL_ENABLE , false); // 4
   Bit_Write(pFlagsAndCommands, CMD_CTTAC_MAKE  , false); // 5
   Bit_Write(pFlagsAndCommands, CMD_CTTAC_BREAK , false); // 6
   Bit_Write(pFlagsAndCommands, CMD_CTTDC_MAKE  , false); // 7
   Bit_Write(pFlagsAndCommands, CMD_CTTDC_BREAK , false); // 8
   Bit_Write(pFlagsAndCommands, CMD_PREDC_START , false); // 9
   Bit_Write(pFlagsAndCommands, CMD_PREDC_STOP  , false); // 10
   Bit_Write(pFlagsAndCommands, CMD_RESET       , false); // 11


    // Run Controls
    switch(*pFSMstate){
    case STT_OFF:{
        stepADCcalibration(&(pOverlordVar->ADCcalVar), gVabc_adc, gIabc_adc);

        gVACcalibration[0] = pOverlordVar->ADCcalVar.VaLPF.output;
        gVACcalibration[1] = pOverlordVar->ADCcalVar.VbLPF.output;
        gVACcalibration[2] = pOverlordVar->ADCcalVar.VcLPF.output;

        gIACcalibration[0] = pOverlordVar->ADCcalVar.IaLPF.output;
        gIACcalibration[1] = pOverlordVar->ADCcalVar.IbLPF.output;
        gIACcalibration[2] = pOverlordVar->ADCcalVar.IcLPF.output;

        break;}
    case STT_FAULT:{
        if (pOverlordVar->prevFSMstate == STT_ON){
            turnAllOff();
            resetRPQctrl(&(pOverlordVar->RPQCvar));
        }
        break;}
    case STT_PRELOAD:{
        preloadReady = true;
        if (!FLG_READY_PREDC){
            preloadReady = false;
        }
        Bit_Write(pFlagsAndCommands, FLG_PREDCOK, preloadReady);
        break;}
    case STT_DCREADY:{
        if (pOverlordVar->prevFSMstate != STT_DCREADY){
            resetGridAnalyser(&(pOverlordVar->GridAnalVar));
        }

        break;}
    case STT_SYNCHING:{
        // Update Grid analyser
        converged = stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);

        if (pOverlordVar->prevFSMstate == STT_ON){
            turnAllOff();
            resetRPQctrl(&(pOverlordVar->RPQCvar));
        }
        break;}
    case STT_STANDBY:{
        // Update Grid analyser
        converged =  stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);

        if (pOverlordVar->prevFSMstate == STT_ON){
            turnAllOff();
            resetRPQctrl(&(pOverlordVar->RPQCvar));
        }
        break;}
    case STT_ON:{
        // Update Grid analyser
        converged = stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);

        if (pOverlordVar->prevFSMstate != STT_ON){
            turnAllOn(EPWM2_BASE);
            turnAllOn(EPWM5_BASE);
            turnAllOn(EPWM6_BASE);
        }
        // Step Power Control

        double P_ref = gPref * 0.1;
        double Q_ref = gQref * 0.1;
        stepRPQctrl(&(pOverlordVar->RPQCvar),
                    Vabc, Iabc, Vdc,
                    P_ref, Q_ref,
                    pgPseqp, pgQseqp,
                    GridAnalOutputs,
                    dutyCycleABCout);

        break;}
    default:{
        break;}
    }


    *pgGridAnalysis = GridAnalOutputs;



    // Update PWM's
    uint16_t counterValues_abc[3];
    dutyCycleConversion(dutyCycleABCout, counterValues_abc);

    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[0]);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[0]);

    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[1]);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[1]);

    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[2]);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[2]);


    //    Write GPO's
    GPIO_writePin(PREDC,        Bit_Read(*pOutputVector, OUT_PREDC));
    GPIO_writePin(CNTCT_AC_PWR, Bit_Read(*pOutputVector, OUT_CTTAC_pwr));
    GPIO_writePin(GPR1,         Bit_Read(*pOutputVector, OUT_CTTAC_sig));
    GPIO_writePin(CNTCT_DC,     Bit_Read(*pOutputVector, OUT_CTTDC));
    GPIO_writePin(GPR2,         Bit_Read(*pOutputVector, OUT_FAN));

}


