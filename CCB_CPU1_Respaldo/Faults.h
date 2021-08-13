// FAULT FLAGS BITS
#define FLT_OVERCURRENT 0 // +1
#define FLT_OVERTEMP    1 // +2
#define FLT_OVERVOLTAGE 2 // +4
#define FLT_NOGRID      3 // +8
#define FLT_VDCERR      4 // +16
#define FLT_NOSUPERV    5 // +32
#define FLT_GROUND_AC   6
#define FLT_GROUND_DC   7
#define FLT_COMUNIC_CAN 8
#define FLT_EMERG_STOP  9
#define FLT_TIMEOUT_CAN 10

// WARNING FLAGS BITS
#define WRNG_CAN_NO_RQST    0

// Definition of input bits
// Internal flags (FLG)
#define FLG_FAULT   0
#define FLG_PLLOK   1
#define FLG_PREDCOK 2


// FAULT THRESHOLD VALUES
#define MAX_AC_CURRENT  (15.0*SQ2) // A peak
#define MAX_GND_CURRENT 3.0       // A peak

#define MAX_DC_VOLTAGE 200.0  // V
#define MIN_DC_VOLTAGE 100.0  // V

//#define MAX_AC_VOLTAGE (V_FN_RMS_NOM * SQ2 * 1.5) // V ph-n peak
//#define MIN_AC_VOLTAGE (V_FN_RMS_NOM * SQ2 * 0.7) // V ph-n peak

#define MAX_AC_VOLTAGE (250.0 * SQ2) // V ph-n peak
#define MIN_AC_VOLTAGE  (30.0 * SQ2) // V ph-n peak




void resetFaults(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands){
    (*pFaultVector) = 0;
    Bit_Write(pFlagsAndCommands, FLG_FAULT, false);
}

void testVdcLimInf(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double Vdc){

    if(Vdc < MIN_DC_VOLTAGE){
        Bit_Write(pFaultVector, FLT_VDCERR, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
    }
}

void testVdcLimSup(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double Vdc){

    if(Vdc > MAX_DC_VOLTAGE){
        Bit_Write(pFaultVector, FLT_VDCERR, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
    }
}

void testIac(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double *Iabc){

    if(abs(Iabc[0]) > MAX_AC_CURRENT ||
       abs(Iabc[1]) > MAX_AC_CURRENT ||
       abs(Iabc[2]) > MAX_AC_CURRENT)
       {
        Bit_Write(pFaultVector, FLT_OVERCURRENT, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
       }
}

void testGround(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double *Iabc){

    if(fabs(Iabc[0] + Iabc[1] + Iabc[2]) > MAX_GND_CURRENT)
    {
        Bit_Write(pFaultVector, FLT_GROUND_AC, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
    }
}

void testVac(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double *Vabc){

    if(fabs(Vabc[0]) > MAX_AC_VOLTAGE ||
       fabs(Vabc[1]) > MAX_AC_VOLTAGE ||
       fabs(Vabc[2]) > MAX_AC_VOLTAGE)
       {
        Bit_Write(pFaultVector, FLT_OVERVOLTAGE, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
       }
}

void testNoGrid(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, double Vfnrms_seqp_estimate){

    if(Vfnrms_seqp_estimate * SQ2 < MIN_AC_VOLTAGE){
        Bit_Write(pFaultVector, FLT_NOGRID, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
    }

}

bool MessageCanFail(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands, uint32_t TrySend)
{
    bool sendOK = true;
    uint16_t repeatByShiment = 1200; //for a data send, iterate 1200 times approx.
    if (TrySend >= (MAX_SEND_CAN * repeatByShiment))
    {
        Bit_Write(pFaultVector, FLT_COMUNIC_CAN, true);
        Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
        sendOK = false;
    }
    return sendOK;
}

void EmergencyStop(uint16_t *pFaultVector, uint16_t *pFlagsAndCommands)
{
    Bit_Write(pFaultVector, FLT_EMERG_STOP, true);
    Bit_Write(pFlagsAndCommands, FLG_FAULT, true);
}


