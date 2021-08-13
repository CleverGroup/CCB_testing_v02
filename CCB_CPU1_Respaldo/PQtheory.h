
void instantPwr(double *Vabg, double *Iabg, double *instPwr_results, bool AdjustPowerVariance){

    // AdjustPowerVariance = 0, 1. select 1 if Vabg and Iabg come from the powerVARIANT clarke transforms;


    // power_results = {p, q, p0}
    instPwr_results[0] = Vabg[0] * Iabg[0] + Vabg[1] * Iabg[1];
    instPwr_results[1] = Vabg[1] * Iabg[0] - Vabg[0] * Iabg[1];
    instPwr_results[2] =                                        Vabg[2] * Iabg[2];

    if (AdjustPowerVariance){
        double scalePQ = 3.0/2.0;
        double scaleP0 = 3.0;

        instPwr_results[0] *= scalePQ;
        instPwr_results[1] *= scalePQ;
        instPwr_results[2] *= scaleP0;
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////

struct PowerAnalyzer{

    // OUTPUTS
    double avgPQp0[3];  // average real power, imaginary power, zero-sequence power
    double oscPQp0[3];  // oscilating real power, imaginary power, zero-sequence power

    struct LPFN* PQp0Filters;
};

void initPwrAnalyzer(struct PowerAnalyzer *PAvar,
                        double Ts, uint8_t order, double cutoff_freq){

    int i;
    for(i = 0; i < 3; i++){
        initLPFN(&(PAvar->PQp0Filters[i]), order, Ts, cutoff_freq, 0.0);
    }
}

void stepPwrAnalyzer(struct PowerAnalyzer *PAvar, double *Vabc, double *Iabc){

    // transform to abg
    double Vabg[3];
    double Iabg[3];
    fwdClarkePI(Vabc, Vabg);
    fwdClarkePI(Iabc, Iabg);

    //compute instantaneous components
    double PQp0_inst[3];
    instantPwr(Vabg, Iabg, PQp0_inst, false);

    int i;
    for(i = 0; i < 3; i++){
        PAvar->avgPQp0[i] = stepLPFN(&(PAvar->PQp0Filters[i]), PQp0_inst[i]);
        PAvar->oscPQp0[i] = PQp0_inst[i] - PAvar->avgPQp0[i];
    }
}






