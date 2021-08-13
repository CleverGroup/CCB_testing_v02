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
// PWM OUTPUT CONTROL

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


////////////////////////////////////////////////////////////////////////////////////////////////


struct RobustPQctrl{

    struct PQctrl PQCblock;
    struct HigherHarmonicCompensator HHCblock;

    struct SlewRateLimiter PwrSRL;
    struct SlewRateLimiter QwrSRL;

};

void initRPQctrl(struct RobustPQctrl *pRPQCvar, double Ts,
             double kpC, double kiC, double nominalReactance, double beta_d_eq,
             double kpP, double kiP,
             double kpQ, double kiQ,
             double VDCcutoff_freq, double VDCinit,
             int harmonic1, double kpH1, double krH1,
             int harmonic2, double kpH2, double krH2){

    initPQctrl(&(pRPQCvar->PQCblock), Ts, kpC, kiC, nominalReactance, beta_d_eq, kpP, kiP, kpQ, kiQ, VDCcutoff_freq, VDCinit);
    initHHC(&(pRPQCvar->HHCblock), Ts, harmonic1, kpH1, krH1, harmonic2, kpH2, krH2);
    initSRL(&(pRPQCvar->PwrSRL), Ts, SR_P, 0.0); // se hardcodeó SR_P, arreglar pasando prámetro
    initSRL(&(pRPQCvar->QwrSRL), Ts, SR_Q, 0.0); // se hardcodeó SR_Q, arreglar pasando prámetro

    return;
}

void resetRPQctrl(struct RobustPQctrl *pRPQCvar){

    resetPQctrl(&(pRPQCvar->PQCblock));
    resetHHC(&(pRPQCvar->HHCblock));
    resetSRL(&(pRPQCvar->PwrSRL), 0.0);
    resetSRL(&(pRPQCvar->QwrSRL), 0.0);

    return;
}

void stepRPQctrl(struct RobustPQctrl *pRPQCvar,
                double *Vabc, double *Iabc, double Vdc,
                double *PQ_seqp_ref,
                double *pgPseqp, double *pgQseqp,
                struct GA_results GridAnalOutputs,
                struct StateVariableMonitor *pgStateMonitor,
                double *DutyCycle_ABC_out){

    // step forward in time, i.e. update state

    double Iabg[3];
    fwdClarkePV(Iabc, Iabg);

    double omega = GridAnalOutputs.omega;
    double theta = GridAnalOutputs.theta;

    double Vabg_seqp[3];
    Vabg_seqp[0] = GridAnalOutputs.alpha_seqp;
    Vabg_seqp[1] = GridAnalOutputs.beta_seqp;
    Vabg_seqp[2] = 0.0;

    double Vdq0_seqp[3];
    fwdParkQ(Vabg_seqp, theta, Vdq0_seqp);

    double Vabg_seqn[3];
    Vabg_seqn[0] = GridAnalOutputs.alpha_seqn;
    Vabg_seqn[1] = GridAnalOutputs.beta_seqn;
    Vabg_seqn[2] = 0.0;

    // PQ Powers and DQ Currents Controller
    double Idq0[3];
    fwdParkQ(Iabg, theta, Idq0);
    memcpy(pgStateMonitor->Idq0_inst, Idq0, sizeof(Idq0));

    double instPQP0_seqp[3];
    bool adjustPowerVariance = true;
    instantPwr(Vabg_seqp, Iabg, instPQP0_seqp, adjustPowerVariance);
    *pgPseqp = instPQP0_seqp[0];
    *pgQseqp = instPQP0_seqp[1];
    memcpy(pgStateMonitor->PQp0_seqp_inst, instPQP0_seqp, sizeof(instPQP0_seqp));

    double slewedPQref[2];
    slewedPQref[0] = stepSRL(&(pRPQCvar->PwrSRL), PQ_seqp_ref[0]); // use pending
    slewedPQref[1] = stepSRL(&(pRPQCvar->QwrSRL), PQ_seqp_ref[1]); // use pending

    double DutyCycle_DQ0_out[3];
    double Idq_error[2];
    stepPQctrl(&(pRPQCvar->PQCblock), Idq0, instPQP0_seqp, Vdc, PQ_seqp_ref, DutyCycle_DQ0_out, Idq_error);


    memcpy(pgStateMonitor->beta_dq0, DutyCycle_DQ0_out, sizeof(DutyCycle_DQ0_out));
    pgStateMonitor->Idq0_ref[0] = pRPQCvar->PQCblock.activePI.output;
    pgStateMonitor->Idq0_ref[1] = pRPQCvar->PQCblock.reactivePI.output;
    pgStateMonitor->Idq0_ref[2] = 0.0;

    ///////// FORZAR PWM ///////////
//    DutyCycle_DQ0_out[0] = BETA_D_EQ;
//    DutyCycle_DQ0_out[1] = 0.0;
//    DutyCycle_DQ0_out[2] = 0.0;
    ///////// FORZAR PWM ///////////

    // Higher Harmonic compensation
    bool enable_HHC = true;
    if (enable_HHC){
        double beta_dq_HHC[2];
        stepHHC(&(pRPQCvar->HHCblock), Idq_error, theta, omega, beta_dq_HHC);
        DutyCycle_DQ0_out[0] += beta_dq_HHC[0];
        DutyCycle_DQ0_out[1] += beta_dq_HHC[1];
    }

    double DutyCycle_ABG_out[3];
    revParkQ(DutyCycle_DQ0_out, theta, DutyCycle_ABG_out);

    // scaling factor from refVoltage to refDutyCycle based on filtered Vdc
    double Vdc_2 = 0.5 * pRPQCvar->PQCblock.DQCvar.Vdc_filt;
    pgStateMonitor->Vdc_filtered = pRPQCvar->PQCblock.DQCvar.Vdc_filt;

    // Dips compensator
    bool enable_seqn_FF = true;
    if (enable_seqn_FF){
        DutyCycle_ABG_out[0] -= Vabg_seqn[0] / Vdc_2;
        DutyCycle_ABG_out[1] -= Vabg_seqn[1] / Vdc_2;
    }

    SVM(DutyCycle_ABG_out, DutyCycle_ABC_out);

    return;
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

void initOverlord(struct Overlord *pOverlordVar){


    initRPQctrl(&(pOverlordVar->RPQCvar),
                TS,
                KP_PI_CURRENT, KI_PI_CURRENT, NOMINAL_REACTANCE, BETA_D_EQ,
                KP_PI_P, KI_PI_P, KP_PI_Q, KI_PI_Q,
                VDC_LPF_CUTOFF, VDC_NOM,
                HARMONIC1, KP_PR_H1, KR_PR_H1,
                HARMONIC2, KP_PR_H1, KR_PR_H2);

    initGridAnalyser(&(pOverlordVar->GridAnalVar),
                   TS, KP_PI_PLL, KI_PI_PLL,
                   SOGIGAIN, WG_NOM, V_FN_RMS_NOM,
                   PLL_ERROR_THRESH, PLL_ERROR_LPF_TC, PLL_ERROR_LPF_INIT);

    initCCBFSM(&(pOverlordVar->mainFSM));

    pOverlordVar->FSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->prevFSMstate = pOverlordVar->mainFSM.currentState;
    pOverlordVar->onEntry = false;

    initPLctrl(&(pOverlordVar->PreDCvar), VDC_NOM, VDC_PRELOAD_THRESH, VDC_MIN);
    initADCcalibration(&(pOverlordVar->ADCcalVar), TS, OFFSET_CALIBRATION_LPF_FREQ, OFFSET_CALIBRATION_LPF_INIT);

}

void resetOverlord(struct Overlord *pOverlordVar,
                   uint16_t *pFlagsAndCommands,
                   uint16_t *pOutputVector,
                   uint16_t *pFaultVector){


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


void stepOverlord(struct Overlord *pOverlordVar,
                  uint16_t *pFlagsAndCommands, uint16_t *pOutputVector, uint16_t *pFaultVector, uint16_t *pFSMstate,
                  uint16_t *gVabc_adc, uint16_t *gIabc_adc, uint16_t gVdc_adc,
                  double *gVACcalibration, double *gIACcalibration,
                  double gPref, double gQref,
                  double *pgPseqp, double *pgQseqp,
                  double *pgVdc,
                  struct GA_results *pgGridAnalysis,
                  struct StateVariableMonitor *pgStateMonitor){

    // LOCAL VARIABLES //
    bool converged = false;
    struct GA_results GridAnalOutputs;
    bool preDCOK = false;
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
    pgStateMonitor->Vdc         = Vdc;
    memcpy(pgStateMonitor->Iabc_inst, Iabc, sizeof(Iabc));
    memcpy(pgStateMonitor->Vabc_inst, Vabc, sizeof(Vabc));



    // CHECK FAULTS
    if (pOverlordVar->FSMstate > STT_PRELOAD){
        testVdcLimSup(pFaultVector, pFlagsAndCommands, Vdc); // no hay que revisar límite inferior en estado de precarga
    }
    if (pOverlordVar->FSMstate > STT_DCREADY){
        testVdcLimInf(pFaultVector, pFlagsAndCommands, Vdc);
    }
    if (pOverlordVar->FSMstate > STT_SYNCHING){
        testVac(pFaultVector, pFlagsAndCommands, Vabc);
    }
    if (pOverlordVar->FSMstate > STT_STANDBY){
        testNoGrid(pFaultVector, pFlagsAndCommands, (pgGridAnalysis->Vfnrms_seqp));
        testIac(pFaultVector, pFlagsAndCommands, Iabc);
    }


    // UPDATE FSM
    pOverlordVar->prevFSMstate = pOverlordVar->FSMstate;
    pOverlordVar->FSMstate = stepFSM(&(pOverlordVar->mainFSM), *pFlagsAndCommands, pOutputVector);
    pOverlordVar->onEntry = (pOverlordVar->FSMstate != pOverlordVar->prevFSMstate);

    *pFSMstate = pOverlordVar->FSMstate;

    preDCOK = Bit_Read(*pFlagsAndCommands, CMD_CTTDC_MAKE);

    // CLEAR ALL COMMANDS
    Bit_Write(pFlagsAndCommands, CMD_CTRL_DISABLE, false); // 3
    Bit_Write(pFlagsAndCommands, CMD_CTRL_ENABLE , false); // 4
    Bit_Write(pFlagsAndCommands, CMD_CTTAC_MAKE  , false); // 5
    Bit_Write(pFlagsAndCommands, CMD_CTTAC_BREAK , false); // 6
    Bit_Write(pFlagsAndCommands, CMD_CTTDC_MAKE  , false); // 7
    Bit_Write(pFlagsAndCommands, CMD_CTTDC_BREAK , false); // 8
    Bit_Write(pFlagsAndCommands, CMD_PREDC_START , false); // 9
    Bit_Write(pFlagsAndCommands, CMD_PREDC_STOP  , false); // 10
    Bit_Write(pFlagsAndCommands, CMD_RESET       , false); // 11


    // RESET CONTROLS
    if(pOverlordVar->onEntry){

        if(pOverlordVar->prevFSMstate == STT_ON){
            turnAllOff();
            resetRPQctrl(&(pOverlordVar->RPQCvar));
        }

        if(pOverlordVar->FSMstate == STT_SYNCHING){
            resetGridAnalyser(&(pOverlordVar->GridAnalVar));
        }

        if(pOverlordVar->prevFSMstate == STT_FAULT){
            resetOverlord(pOverlordVar, pFlagsAndCommands, pOutputVector, pFaultVector);
        }
    }


    // RUN CONTROLS
    if(pOverlordVar->FSMstate == STT_OFF){

        stepADCcalibration(&(pOverlordVar->ADCcalVar), gVabc_adc, gIabc_adc);

        gVACcalibration[0] = pOverlordVar->ADCcalVar.VaLPF.output;
        gVACcalibration[1] = pOverlordVar->ADCcalVar.VbLPF.output;
        gVACcalibration[2] = pOverlordVar->ADCcalVar.VcLPF.output;

        gIACcalibration[0] = pOverlordVar->ADCcalVar.IaLPF.output;
        gIACcalibration[1] = pOverlordVar->ADCcalVar.IbLPF.output;
        gIACcalibration[2] = pOverlordVar->ADCcalVar.IcLPF.output;
    }
    else if(pOverlordVar->FSMstate == STT_PRELOAD){

//        preDCOK = stepPLctrl(&(pOverlordVar->PreDCvar), Vdc);

        Bit_Write(pFlagsAndCommands, FLG_PREDCOK, preDCOK);
    }
    else if(pOverlordVar->FSMstate == STT_DCREADY){

        if(pOverlordVar->onEntry){
            resetGridAnalyser(&(pOverlordVar->GridAnalVar));
        }
    }
    else if(pOverlordVar->FSMstate == STT_SYNCHING){

        converged = stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);
    }
    else if(pOverlordVar->FSMstate == STT_STANDBY){

        converged = stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);
    }
    else if(pOverlordVar->FSMstate == STT_ON){

        converged = stepGridAnalyser(&(pOverlordVar->GridAnalVar), Vabc, &GridAnalOutputs);
        Bit_Write(pFlagsAndCommands, FLG_PLLOK, converged);

        if(pOverlordVar->onEntry){
            turnAllOn(EPWM2_BASE);
            turnAllOn(EPWM5_BASE);
            turnAllOn(EPWM6_BASE);
        }

        double PQ_seqp_ref[2];
        PQ_seqp_ref[0] = gPref * 0.1;
        PQ_seqp_ref[1] = gQref * 0.1;
//        PQ_seqp_ref = pgStateMonitor->PQ_seqp_ref * 0.1;

        stepRPQctrl(&(pOverlordVar->RPQCvar),
                    Vabc, Iabc, Vdc,
                    PQ_seqp_ref,
                    pgPseqp, pgQseqp,
                    GridAnalOutputs,
                    pgStateMonitor,
                    dutyCycleABCout);
    }


    *pgGridAnalysis = GridAnalOutputs;
    memcpy(pgStateMonitor->beta_abc, dutyCycleABCout, sizeof(dutyCycleABCout));


    // UPDATE PWM's
    uint16_t counterValues_abc[3];
    dutyCycleConversion(dutyCycleABCout, counterValues_abc);

    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[0]);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[0]);

    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[1]);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[1]);

    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, counterValues_abc[2]);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, counterValues_abc[2]);


    //    WRITE GPO's
    GPIO_writePin(PREDC,        Bit_Read(*pOutputVector, OUT_PREDC));
    GPIO_writePin(CNTCT_AC_PWR, Bit_Read(*pOutputVector, OUT_CTTAC_pwr));
    GPIO_writePin(GPR1,         Bit_Read(*pOutputVector, OUT_CTTAC_sig));
    GPIO_writePin(CNTCT_DC,     Bit_Read(*pOutputVector, OUT_CTTDC));
    GPIO_writePin(GPR2,         Bit_Read(*pOutputVector, OUT_FAN));

}


