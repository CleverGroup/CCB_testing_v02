

struct LPF1{

    // 1st ORDER IIR LOW PASS FILTER

    double Ts; // Sampling Period
    double a;  // filter parameter
    double output;
};

void initLPF1(struct LPF1 *pLPFvar, double Ts, double cutoff_freq, double initialOut){

    pLPFvar->Ts = Ts;

    double numerator = _2PI * cutoff_freq * Ts;
    pLPFvar->a = numerator / (numerator + 1.0);

    pLPFvar->output = initialOut;
}

void resetLPF1(struct LPF1 *pLPFvar, double initialOut){

    pLPFvar->output = initialOut;
}

double stepLPF1(struct LPF1 *pLPFvar, double input){

    // step forward in time, i.e. update state

    pLPFvar->output = pLPFvar->a * input + (1.0 - pLPFvar->a) * pLPFvar->output;

    return pLPFvar->output;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct LPFN{

    // Nth order IIR low pass filter

    uint8_t order;
    double out;

    struct LPF1* filters;

};


void initLPFN(struct LPFN *pLPFNvar, uint8_t order, double Ts, double cutoff_freq, double initialOut){

    pLPFNvar->order = order;
    pLPFNvar->out = initialOut;

    pLPFNvar->filters = malloc(order * sizeof(struct LPF1));

    int i;
    for(i = 0; i < pLPFNvar->order; i++){
        initLPF1(&(pLPFNvar->filters[i]), Ts, cutoff_freq, initialOut);
    }
}

void resetLPFN(struct LPFN *pLPFNvar, double initialOut){

    pLPFNvar->out = initialOut;

    int i;
    for(i = 0; i < pLPFNvar->order; i++){
        resetLPF1(&(pLPFNvar->filters[i]), initialOut);
    }
}

double stepLPFN(struct LPFN *pLPFNvar, double input){

    int i;
    double intermediateOutput = input;
    for(i = 0; i < pLPFNvar->order; i++){
        intermediateOutput = stepLPF1(&(pLPFNvar->filters[i]), intermediateOutput);
    }

    pLPFNvar->out = intermediateOutput;

    return pLPFNvar->out;
}

////////////////////////////////////////////////////////////////////////////////////////////////


struct SlewRateLimiter{

    double Ts;          // Sampling Period
    double maxDelta;    // filter parameter
    double output;      // state
};

void initSRL(struct SlewRateLimiter *pSRLvar, double Ts, double max_d_dt, double initialOut){

    pSRLvar->Ts = Ts;

    pSRLvar->maxDelta = max_d_dt * Ts;

    pSRLvar->output = initialOut;
}

void resetSRL(struct SlewRateLimiter *pSRLvar, double initialOut){

    (*pSRLvar).output = initialOut;
}

double stepSRL(struct SlewRateLimiter *pSRLvar, double input){

    // step forward in time, i.e. update state
    double delta = input - pSRLvar->output;
    delta = MAX(- pSRLvar->maxDelta, delta);
    delta = MIN(+ pSRLvar->maxDelta, delta);

    pSRLvar->output = delta;

    return pSRLvar->output;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct HPF1{

    // 1st ORDER HIGH PASS FILTER
    // built around concept: HPF = 1-LPF

    struct LPF1 innerLPF;
    double output;
};

void initHPF1(struct HPF1 *pHPFvar, double Ts, double cutoff_freq, double initialOut){

    initLPF1(&(pHPFvar->innerLPF), Ts, cutoff_freq, 0.0);
    pHPFvar->output = initialOut;
}

void resetHPF1(struct HPF1 *pHPFvar, double initialOut){

    resetLPF1(&(pHPFvar->innerLPF), 0.0);
    pHPFvar->output = initialOut;
}

double stepHPF1(struct HPF1 *pHPFvar, double input){

    // step forward in time, i.e. update state

    pHPFvar->output = input - stepLPF1(&(pHPFvar->innerLPF), input);
    return pHPFvar->output;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct NotchFtr{

    // Variable Frequency Notch Filter
    double Ts;
    double rho;

    double rho2;

    double prevOut;
    double prevprevOut;

    double prevIn;
    double prevprevIn;

};

void resetNotch(struct NotchFtr *pNotchVar, double initialState){

    pNotchVar->prevIn       = initialState;
    pNotchVar->prevprevIn   = initialState;
    pNotchVar->prevOut      = initialState;
    pNotchVar->prevprevOut  = initialState;

}

void initNotch(struct NotchFtr *pNotchVar, double Ts, double rho, double initialState){

    pNotchVar->Ts = Ts;
    pNotchVar->rho = rho;

    pNotchVar->rho2 = rho*rho;

    resetNotch(pNotchVar, initialState);
}

double stepNotch(struct NotchFtr *pNotchVar, double input, double w0){

    // step forward in time, i.e. update state
    // transfer function:
    // N(z) = (1 - 2*c*z^-1 + z^-2) / (1 - 2*r*c*z^-1 + r^2*z^-2)
    // with c = cos(w0*Ts)


    double _2c = 2.0 * cos(w0 * pNotchVar->Ts);

    double out = 0.0;

    out += _2c * (pNotchVar->rho) * (pNotchVar->prevOut);
    out -= (pNotchVar->rho2) * (pNotchVar->prevprevOut);

    out += input;
    out -= _2c * (pNotchVar->prevIn);
    out += (pNotchVar->prevprevIn);

    pNotchVar->prevprevIn = pNotchVar->prevIn;
    pNotchVar->prevIn = input;

    pNotchVar->prevprevOut = pNotchVar->prevOut;
    pNotchVar->prevOut = out;

    return out;

}

////////////////////////////////////////////////////////////////////////////////////////////////

struct NotchN{

    // Harmonically related adaptive notch filters

    struct NotchFtr* filters;

    uint8_t order;
    double out;

};

void initNotchN(struct NotchN *pNchNvar, uint8_t order, double Ts, double rho, double initialState){

    pNchNvar->order = order;
    pNchNvar->out = 0.0;

    pNchNvar->filters = malloc(order * sizeof(struct NotchFtr));

    int i;
    for(i = 0; i < pNchNvar->order; i++){
        initNotch(&(pNchNvar->filters[i]), Ts, rho, initialState);
    }
}

void resetNotchN(struct NotchN *pNchNvar, double initialState){

    int i;
    for(i = 0; i < pNchNvar->order; i++){
        resetNotch(&(pNchNvar->filters[i]), initialState);
    }
}

double stepNotchN(struct NotchN *pNchNvar, double input, double w0){

    double output = input;

    int i;
    for(i = 0; i < pNchNvar->order; i++){
        output = stepNotch(&(pNchNvar->filters[i]), output, w0*(i+1));
    }

    pNchNvar->out = output;

    return output;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct PIctrl{

    // Proportional Integral Controller

    double kp;    // Proportional Gain
    double ki;    // Integral Gain
    double Ts;    // Sampling Period

    bool satEnabled;  // {1,0} saturation of output enabled
    double _max;     // maximum output value
    double _min;     // minimum output value

    double int_action;
    bool clamping;

    double output;
};

void initPI(struct PIctrl *pPIvar,
            double kp,
            double ki,
            double Ts,
            bool satEnabled,
            double minOutput,
            double maxOutput,
            double initialIntegral){

    pPIvar->kp = kp;
    pPIvar->ki = ki;
    pPIvar->Ts = Ts;

    pPIvar->satEnabled = satEnabled;
    pPIvar->_min = minOutput;
    pPIvar->_max = maxOutput;
    pPIvar->clamping = false;

    pPIvar->int_action = initialIntegral;

    pPIvar->output = initialIntegral;

    return;
}

void resetPI(struct PIctrl *pPIvar, double initialIntegral){

    pPIvar->int_action = initialIntegral;
    pPIvar->clamping = false;
    return;
}

double stepPI(struct PIctrl *pPIvar, double error){

    // step forward in time, i.e. update state

    double prop_action = pPIvar->kp * error;

    // integrates only if not currently clamping
    if(!pPIvar->clamping){
        pPIvar->int_action += pPIvar->ki * error * pPIvar->Ts;
    }

    // general PI output before saturation
    double PI_output = prop_action + pPIvar->int_action;

    pPIvar->output = PI_output;
    // update clamping state
    if (pPIvar->satEnabled){
        // clamping style anti-windup
        double saturatedOutput = MIN(pPIvar->_max, MAX(pPIvar->_min, PI_output));
        double excessOutput = PI_output - saturatedOutput;

        bool isSaturating = (excessOutput != 0.0);
        bool makingItWorse = false;
        if(isSaturating){
            makingItWorse = ((SIGN(error) * SIGN(pPIvar->ki)) == SIGN(excessOutput));
        }

        pPIvar->clamping = (isSaturating && makingItWorse);
        pPIvar->output = saturatedOutput;
    }

    return pPIvar->output;
}


////////////////////////////////////////////////////////////////////////////////////////////////

//
//struct PIDctrl{
//
//    // Proportional Integral Derivative Controller
//
//    double kp;    // Proportional Gain
//    double ki;    // Integral Gain
//    double kd;    // Derivative Gain
//    double N;     // Derivative filter coefficient
//    double Ts;    // Sampling Period
//
//    bool satEnabled;  // {true, false} saturation of output enabled
//    double _max;      // maximum output value
//    double _min;      // minimum output value
//
//    double int_action;  // integral state
//    double dv_state;    // derivative state
//    bool clamping;      // clamping state
//
//    double output;
//};
//
//void initPID(struct PIDctrl *pPIDvar,
//            double kp,
//            double ki,
//            double kd,
//            double N,
//            double Ts,
//            bool satEnabled,
//            double minOutput,
//            double maxOutput,
//            double initialIntegral){
//
//    pPIDvar->kp = kp;
//    pPIDvar->ki = ki;
//    pPIDvar->kd = kd;
//    pPIDvar->N = N;     // LPF filter cut-off frequency (rad/s)
//    pPIDvar->Ts = Ts;
//
//    pPIDvar->satEnabled = satEnabled;
//    pPIDvar->_min = minOutput;
//    pPIDvar->_max = maxOutput;
//    pPIDvar->clamping = false;
//
//    pPIDvar->int_action = initialIntegral;
//    pPIDvar->dv_state = 0.0;
//
//    pPIDvar->output = initialIntegral;
//
//    return;
//}
//
//void resetPID(struct PIDctrl *pPIDvar, double initialIntegral){
//
//    pPIDvar->int_action = initialIntegral;
//    pPIDvar->dv_state = 0.0;
//    pPIDvar->clamping = false;
//    return;
//}
//
//double stepPID(struct PIDctrl *pPIDvar, double error){
//
//    // step forward in time, i.e. update state
//
//    // PROPORTIONAL BRANCH
//    double prop_action = pPIDvar->kp * error;
//
//    // DERIVATIVE BRANCH
//    // derivative of low-pass-filtered error
//    // transfer function:
//    // Kd * s * N / (s + N)
//    // https://www.youtube.com/watch?v=7dUVdrs1e18
//
//    double dv_filt_error = (error - pPIDvar->dv_state) * pPIDvar->N;
//    pPIDvar->dv_state += dv_filt_error;
//    double dv_action = dv_filt_error * pPIDvar->kd;
//
//    // INTEGRAL BRANCH
//    // integrates only if not currently clamping
//    if(!pPIDvar->clamping){
//        pPIDvar->int_action += pPIDvar->ki * error * pPIDvar->Ts;
//    }
//
//    // general PID output before saturation
//    double pid_output = prop_action + pPIDvar->int_action + dv_action;
//
//    // SATURATION
//    double saturatedOutput = pid_output;
//    // update clamping state
//    if (pPIDvar->satEnabled){
//        // clamping style anti-windup
//        double saturatedOutput = MIN(pPIDvar->_max, MAX(pPIDvar->_min, pid_output));
//        double excessOutput = pid_output - saturatedOutput;
//
//        bool isSaturating = (excessOutput != 0.0);
//        bool makingItWorse = false;
//        if(isSaturating){
//            makingItWorse = ((SIGN(error) * SIGN(pPIDvar->ki)) == SIGN(excessOutput));
//        }
//
//        pPIDvar->clamping = (isSaturating && makingItWorse);
//        pPIDvar->output = saturatedOutput;
//    }
//
//
//    pPIDvar->output = saturatedOutput;
//
//    return pPIDvar->output;
//}


////////////////////////////////////////////////////////////////////////////////////////////////


struct SOGI{

    // Second order generalized integrator used for quadrature signal generation

    double gain;  // SOGI gain
    double Ts;    // Sampling Period

    double integral1; // integrator1 state
    double integral2; // integrator2 state

    double quadrature; // 90º phase shifted signal
};

void initSOGI(struct SOGI *pSOGIvar,
              double sogiGain, double Ts){

    pSOGIvar->gain = sogiGain;
    pSOGIvar->Ts = Ts;

    pSOGIvar->integral1 = 0.0;
    pSOGIvar->integral2 = 0.0;

    pSOGIvar->quadrature = 0.0;

    return;
}

void resetSOGI(struct SOGI *pSOGIvar){

    pSOGIvar->integral1 = 0.0;
    pSOGIvar->integral2 = 0.0;

    pSOGIvar->quadrature = 0.0;

    return;
}

void stepSOGI(struct SOGI *pSOGIvar,
              double signal, double omega_grid, double *output2Array){

    // step forward in time, i.e. update state

    // state update
    double sogi_error = signal - pSOGIvar->integral1;
    pSOGIvar->quadrature = omega_grid * pSOGIvar->integral2;
    double u = pSOGIvar->gain * sogi_error - pSOGIvar->quadrature;
    double v = u * omega_grid;

    pSOGIvar->integral1 += v * pSOGIvar->Ts;
    pSOGIvar->integral2 += pSOGIvar->integral1 * pSOGIvar->Ts;

    // setup outputs
    output2Array[0] = pSOGIvar->integral1;
    output2Array[1] = pSOGIvar->quadrature;

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct SequenceEstimator{

    // Sequence component separator based on double SOGI topology

    double Ts;           // Sampling Period
    double sogiGain;
    double vfn_peak_nom; // nominal phase-neutral peak voltage

    struct SOGI sogi1;
    struct SOGI sogi2;

    double posSeq;
    double negSeq;
};

struct SequenceResults{

    double alpha_seqp;
    double beta_seqp;
    double alpha_seqn;
    double beta_seqn;

};

void initSeqEst(struct SequenceEstimator *pSeqEstvar,
                double Ts, double sogiGain, double vfnRmsNom){

    pSeqEstvar->vfn_peak_nom = vfnRmsNom * SQ2;

    initSOGI(&(pSeqEstvar->sogi1), sogiGain, Ts);
    initSOGI(&(pSeqEstvar->sogi2), sogiGain, Ts);

    pSeqEstvar->posSeq = 0.0;
    pSeqEstvar->negSeq = 0.0;

    return;
}

void resetSeqEst(struct SequenceEstimator *pSeqEstvar){

    resetSOGI(&(pSeqEstvar->sogi1));
    resetSOGI(&(pSeqEstvar->sogi2));

    return;
}

void stepSeqEst(struct SequenceEstimator *pSeqEstvar,
                      double *Vabg, double omega_grid, struct SequenceResults *pSeqResultsVar){

    // step forward in time, i.e. update state

    // Voltage normalization
    double Vabg_norm[3];
    int k = 0;
    for(k = 0; k<3; k++){
        Vabg_norm[k] = Vabg[k] / pSeqEstvar->vfn_peak_nom;
    }

    // SOGI update
    double A_qA[2];
    double B_qB[2];
    stepSOGI(&(pSeqEstvar->sogi1), Vabg_norm[0], omega_grid, A_qA);
    stepSOGI(&(pSeqEstvar->sogi2), Vabg_norm[1], omega_grid, B_qB);

    // Mix and Match of direct and quadrature signals
    pSeqResultsVar->alpha_seqp = 0.5*(A_qA[0] - B_qB[1]); //positive sequence alpha
    pSeqResultsVar->beta_seqp  = 0.5*(A_qA[1] + B_qB[0]); //positive sequence beta
    pSeqResultsVar->alpha_seqn = 0.5*(B_qB[1] + A_qA[0]); //negative sequence alpha
    pSeqResultsVar->beta_seqn  = 0.5*(B_qB[0] - A_qA[1]); //negative sequence beta

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct PLL{

    // Synchronous reference frame based Phase-Locked Loop

    double Ts;                    // Sampling Period

    double wg_nom;                // Grid frequency nominal value (rad/s)
    struct PIctrl pi;             // PI regulator
    double thg_estimate;          // Grid angle estimate (rad)
    double wg_estimate;           // Grid frequency estimate (rad/s)
    double Vfnrms_seqp_estimate;  // Estimated phase-neutral rms positive sequence voltage

    struct LPFN wgLPF;            // omega filtering
    double wg_f;                  // filtered omega estimate

    struct NotchN qNotch;         // error notch

    double error;                 // Synchronization error
    struct LPF1 errorLPF;         // error filtering
    double error_thresh;          // maximum allowable filtered error
    double error_lpf_init;        // initial filtered error

    uint8_t converged;                // state variable dependent on filtered error

};

struct PLL_results{

    double omega;                   // Grid frequency estimate (rad/s)
    double theta;                   // Grid angle estimate (rad)
    double Vfnrms_seqp_normalized;  // Estimated phase-neutral rms positive sequence voltage
    double error;                   // Synchronization error

};

void initPLL(struct PLL *pPLLvar,
             double Ts, double kp, double ki, double wg_nom, double error_thresh, double error_timeCons, double error_lpf_init){

    pPLLvar->Ts = Ts;

    initPI(&(pPLLvar->pi), kp, ki, Ts, false, 0.0, 0.0, wg_nom);

    pPLLvar->wg_nom = wg_nom;
    pPLLvar->thg_estimate = 0.0;
    pPLLvar->wg_estimate = wg_nom;
    pPLLvar->Vfnrms_seqp_estimate = 0.0;

    initLPFN(&(pPLLvar->wgLPF), PLL_OMEGA_LPF_ORDER, Ts, PLL_OMEGA_LPF_FREQ, wg_nom);
    pPLLvar->wg_f = wg_nom;

    initNotchN(&(pPLLvar->qNotch), PLL_QAXIS_NOTCH_NUMBER, Ts, PLL_QAXIS_NOTCH_RHO, 0.0);

    pPLLvar->converged = 0;
    pPLLvar->error = 0.0;
    pPLLvar->error_thresh = error_thresh;
    pPLLvar->error_lpf_init = error_lpf_init;
    initLPF1(&(pPLLvar->errorLPF), Ts, 1.0/(error_timeCons * _2PI), error_lpf_init);

}

void resetPLL(struct PLL *pPLLvar){

    pPLLvar->thg_estimate = 0.0;
    pPLLvar->wg_estimate = pPLLvar->wg_nom;
    pPLLvar->Vfnrms_seqp_estimate = 0.0;

    resetPI(&(pPLLvar->pi), pPLLvar->wg_nom);

    resetLPFN(&(pPLLvar->wgLPF), pPLLvar->wg_nom);
    pPLLvar->wg_f = pPLLvar->wg_nom;

    resetNotchN(&(pPLLvar->qNotch), 0.0);

    pPLLvar->error = 0.0;
    resetLPF1(&(pPLLvar->errorLPF), pPLLvar->error_lpf_init);
    pPLLvar->converged = 0;
}

bool stepPLL(struct PLL *pPLLvar, double alpha_posSeq, double beta_posSeq, struct PLL_results *pPllResultsVar){

    // step forward in time, i.e. update state

    double Vabg[3] = {alpha_posSeq, beta_posSeq, 0.0};
    double Vdq0[3];
    fwdParkQ(Vabg, pPLLvar->thg_estimate, Vdq0);

    pPLLvar->Vfnrms_seqp_estimate = Vdq0[0];                          // direct component is voltage estimate

    pPLLvar->error = stepNotchN(&(pPLLvar->qNotch), Vdq0[1], pPLLvar->wg_f); // quadrature component is proxy for error

    double error_f = stepLPF1(&(pPLLvar->errorLPF), pPLLvar->error);
    if (fabs(error_f) < pPLLvar->error_thresh)                      // for Code Composer
    {
        pPLLvar->converged = true;
    }
    else
    {
        pPLLvar->converged = false;
    }

    // STATE UPDATE
    pPLLvar->wg_estimate = stepPI(&(pPLLvar->pi), pPLLvar->error);
    pPLLvar->wg_f = stepLPFN(&(pPLLvar->wgLPF), pPLLvar->wg_estimate);

    pPLLvar->thg_estimate += pPLLvar->wg_estimate * pPLLvar->Ts;
    pPLLvar->thg_estimate = fmod(pPLLvar->thg_estimate, _2PI);

    // OUTPUT
    pPllResultsVar->omega = pPLLvar->wg_f;
    pPllResultsVar->theta = pPLLvar->thg_estimate;
    pPllResultsVar->Vfnrms_seqp_normalized = pPLLvar->Vfnrms_seqp_estimate;
    pPllResultsVar->error = pPLLvar->error;

    return pPLLvar->converged;
}


////////////////////////////////////////////////////////////////////////////////////////////////

struct GridAnalyser{

    // tandem operation of PLL and Sequence estimator for adaptive and robust estimation.
    // PLL feeds frequency estimation to SequenceEstimator
    // SequenceEstimator feeds positive sequence component to PLL
    double vfn_rms_nom;
    double vfn_peak_nom;

    struct PLL pll;
    struct SequenceEstimator seq;
};

struct GA_results{

    double omega;
    double theta;
    double Vfnrms_seqp;
    double pll_error;

    double alpha_seqp;
    double beta_seqp;
    double alpha_seqn;
    double beta_seqn;

};

void initGridAnalyser(struct GridAnalyser *pGridAnalvar,
                    double Ts, double kp, double ki, double sogiGain, double wg_nom, double vfn_rms_nom,
                    double error_thresh, double error_timeCons, double error_lpf_init){

    (*pGridAnalvar).vfn_rms_nom = vfn_rms_nom;
    (*pGridAnalvar).vfn_peak_nom = vfn_rms_nom * SQ2;

    initPLL(&(pGridAnalvar->pll), Ts, kp, ki, wg_nom, error_thresh, error_timeCons, error_lpf_init);
    initSeqEst(&(pGridAnalvar->seq), Ts, sogiGain, vfn_rms_nom);

}

void resetGridAnalyser(struct GridAnalyser *pGridAnalvar){

    resetPLL(&(pGridAnalvar->pll));
    resetSeqEst(&(pGridAnalvar->seq));

}

bool stepGridAnalyser(struct GridAnalyser *pGridAnalvar, double *Vabc, struct GA_results *pGAResultsVar){

    // step forward in time, i.e. update state

    // compute Clarke projections
    double Vabg[3];
    fwdClarkePV(Vabc, Vabg);

    // update sequence estimation
    struct SequenceResults seqResults;
    stepSeqEst(&(pGridAnalvar->seq), Vabg, pGridAnalvar->pll.wg_estimate, &seqResults);
    // update phase estimation
    struct PLL_results pllResults;
    bool converged = stepPLL(&(pGridAnalvar->pll), seqResults.alpha_seqp, seqResults.beta_seqp, &pllResults);

    if (converged)
    {
        // denormalize voltage estimation
        double Vfnrms_seqp_estimate = pllResults.Vfnrms_seqp_normalized * pGridAnalvar->vfn_rms_nom;

        // denormalize and output estimates
        pGAResultsVar->alpha_seqp = seqResults.alpha_seqp * pGridAnalvar->vfn_peak_nom; // positive sequence alpha
        pGAResultsVar->beta_seqp  = seqResults.beta_seqp  * pGridAnalvar->vfn_peak_nom; // positive sequence beta
        pGAResultsVar->alpha_seqn = seqResults.alpha_seqn * pGridAnalvar->vfn_peak_nom; // negative sequence alpha
        pGAResultsVar->beta_seqn  = seqResults.beta_seqn  * pGridAnalvar->vfn_peak_nom; // negative sequence beta

        pGAResultsVar->omega = pllResults.omega;           // omega grid estimate
        pGAResultsVar->theta = pllResults.theta;           // theta grid estimate
        pGAResultsVar->Vfnrms_seqp = Vfnrms_seqp_estimate;  // RMS phase-neutral positive sequence Voltage estimate
        pGAResultsVar->pll_error = pllResults.error;       // PLL error
    }
    return converged;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct PRctrl{

    // Proportional-Resonant Controller

    double Ts;        // Sampling Period

    int h;            // Harmonic number

    double kr;        // Resonant Gain
    double kp;        // Proportional Gain

    double prevOut;
    double prevprevOut;

    double prevIn;


};

void initPR(struct PRctrl *pPRvar,
            double Ts, int harmonic, double prop_gain, double res_gain){

    pPRvar->Ts = Ts;
    pPRvar->h = harmonic;

    pPRvar->kr = res_gain;
    pPRvar->kp = prop_gain;

    pPRvar->prevOut = 0.0;
    pPRvar->prevprevOut = 0.0;
    pPRvar->prevIn = 0.0;

    return;
}

void resetPR(struct PRctrl *pPRvar){

    pPRvar->prevOut = 0.0;
    pPRvar->prevprevOut = 0.0;
    pPRvar->prevIn = 0.0;

    return;
}

double stepPR(struct PRctrl *pPRvar, double cosw0Ts, double error){

    // step forward in time, i.e. update state
    // with updated info from PLL

    // COMPUTE OUTPUT
    // Resonant term
    double out = 0.0;
    out += 2 * cosw0Ts * pPRvar->prevOut;
    out -= pPRvar->prevprevOut;
    out += error;
    out -= cosw0Ts * pPRvar->prevIn;
    out *= pPRvar->Ts * pPRvar->kr;

    // Proportional term
    out += pPRvar->kp * error;

    // UPDATE STATE
    pPRvar->prevIn = error;
    pPRvar->prevprevOut = pPRvar->prevOut;
    pPRvar->prevOut = out;

    return out;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct HigherHarmonicCompensator{

    // Resonant controller based compensation

    double Ts;
    // for 5th and 7th harmonic compensation (alpha-beta)
    int h1;
    struct PRctrl PR1_A;
    struct PRctrl PR1_B;
    // for 11th and 13th harmonic compensation (alpha-beta)
    int h2;
    struct PRctrl PR2_A;
    struct PRctrl PR2_B;

};

void initHHC(struct HigherHarmonicCompensator *pHHCvar, double Ts,
             int harmonic1, double propGain1, double resGain1,
             int harmonic2, double propGain2, double resGain2){

    pHHCvar->Ts = Ts;

    pHHCvar->h1 = harmonic1;
    initPR(&(pHHCvar->PR1_A), Ts, harmonic1, propGain1, resGain1);
    initPR(&(pHHCvar->PR1_B), Ts, harmonic1, propGain1, resGain1);

    pHHCvar->h2 = harmonic2;
    initPR(&(pHHCvar->PR2_A), Ts, harmonic2, propGain2, resGain2);
    initPR(&(pHHCvar->PR2_B), Ts, harmonic2, propGain2, resGain2);

    return;
}

void resetHHC(struct HigherHarmonicCompensator *pHHCvar){

    resetPR(&(pHHCvar->PR1_A));
    resetPR(&(pHHCvar->PR1_B));

    resetPR(&(pHHCvar->PR2_A));
    resetPR(&(pHHCvar->PR2_B));

    return;
}

void stepHHC(struct HigherHarmonicCompensator *pHHCvar,
             double *Idq_error, double theta_grid, double omega_grid, double *beta_dq_HHC){

    // step forward in time, i.e. update state

    double c1 = cos((double)(pHHCvar->h1) * omega_grid * pHHCvar->Ts);
    double c2 = cos((double)(pHHCvar->h2) * omega_grid * pHHCvar->Ts);

    double d1 = stepPR(&(pHHCvar->PR1_A), c1, Idq_error[0]);
    double q1 = stepPR(&(pHHCvar->PR1_B), c1, Idq_error[1]);

    double d2 = stepPR(&(pHHCvar->PR2_A), c2, Idq_error[0]);
    double q2 = stepPR(&(pHHCvar->PR2_B), c2, Idq_error[1]);

    beta_dq_HHC[0] = d1 + d2;
    beta_dq_HHC[1] = q1 + q2;

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct DQctrl{

    // Direct-Quadrature based current controller

    // Fast loops
    struct PIctrl directPI;
    struct PIctrl quadraturePI;

    // Current Reference Compensation Filter
    struct LPF1 directLPF;
    struct LPF1 quadratureLPF;

    // For Decoupling
    double nominalReactance;

    // Optional filter for DC voltage measurement
    struct LPF1 DCvoltageLPF;
    double Vdc_filt;

    // Reset Values
    double beta_d_eq;
    double Vdc_init;

};

void initDQctrl(struct DQctrl *pDQCvar, double Ts,
             double kpC, double kiC, double nominalReactance, double beta_d_eq,
             double VDCcutoff_freq, double VDCinit){



    pDQCvar->beta_d_eq = beta_d_eq;

    pDQCvar->Vdc_init = VDCinit;

    initPI(&(pDQCvar->directPI), kpC, kiC, Ts, false, 0.0, 0.0, pDQCvar->beta_d_eq);
    initPI(&(pDQCvar->quadraturePI), kpC, kiC, Ts, false, 0.0, 0.0, 0.0);

    double TiC = kpC / kiC;
    double cutoff_freq = 1.0/(TiC * _2PI);
    initLPF1(&(pDQCvar->directLPF), Ts, cutoff_freq, 0.0);
    initLPF1(&(pDQCvar->quadratureLPF), Ts, cutoff_freq, 0.0);

    pDQCvar->nominalReactance = nominalReactance;

    initLPF1(&(pDQCvar->DCvoltageLPF), Ts, VDCcutoff_freq, VDCinit);
    pDQCvar->Vdc_filt = VDCinit;


    return;
}

void resetDQctrl(struct DQctrl *pDQCvar){

    resetPI(&(pDQCvar->directPI), pDQCvar->beta_d_eq);
    resetPI(&(pDQCvar->quadraturePI), 0.0);

    resetLPF1(&(pDQCvar->directLPF), 0.0);
    resetLPF1(&(pDQCvar->quadratureLPF), 0.0);

    resetLPF1(&(pDQCvar->DCvoltageLPF), pDQCvar->Vdc_init);
    pDQCvar->Vdc_filt = pDQCvar->Vdc_init;

    return;
}

void stepDQctrl(struct DQctrl *pDQCvar,
             double *Idq0_inst,
             double VDC_inst,
             double *Idq_ref,
             double *beta_dq_h1_seqp){

    // step forward in time, i.e. update state

    pDQCvar->Vdc_filt = stepLPF1(&(pDQCvar->DCvoltageLPF), VDC_inst);
    double K_decoup = pDQCvar->nominalReactance / (pDQCvar->Vdc_filt / 2.0);

    // filter reference
    double Idq_ref_filt[2];
    Idq_ref_filt[0] = stepLPF1(&(pDQCvar->directLPF),       Idq_ref[0]);
    Idq_ref_filt[1] = stepLPF1(&(pDQCvar->quadratureLPF),   Idq_ref[1]);

    // Modulating signals from fast current loops
    beta_dq_h1_seqp[0] = stepPI(&(pDQCvar->directPI),       Idq_ref_filt[0] - Idq0_inst[0]);
    beta_dq_h1_seqp[1] = stepPI(&(pDQCvar->quadraturePI),   Idq_ref_filt[1] - Idq0_inst[1]);

    // Decoupling
    beta_dq_h1_seqp[0] -= K_decoup * Idq0_inst[1];
    beta_dq_h1_seqp[1] += K_decoup * Idq0_inst[0];

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct PQctrl{

    // Power controller

    // Slow power loops
    struct PIctrl activePI;
    struct PIctrl reactivePI;

    // inner current loops
    struct DQctrl DQCvar;


};

void initPQctrl(struct PQctrl *pPQCvar, double Ts,
             double kpC, double kiC, double nominalReactance, double beta_d_eq,
             double kpP, double kiP,
             double kpQ, double kiQ,
             double VDCcutoff_freq, double VDCinit){


    initPI(&(pPQCvar->activePI), kpP, kiP, Ts, false, 0.0, 0.0, 0.0);
    initPI(&(pPQCvar->reactivePI), kpQ, kiQ, Ts, false, 0.0, 0.0, 0.0);
    initDQctrl(&(pPQCvar->DQCvar), Ts, kpC, kiC, nominalReactance, beta_d_eq, VDCcutoff_freq, VDCinit);

    return;
}

void resetPQctrl(struct PQctrl *pPQCvar){

    resetPI(&(pPQCvar->activePI), 0.0);
    resetPI(&(pPQCvar->reactivePI), 0.0);
    resetDQctrl(&(pPQCvar->DQCvar));

    return;
}

void stepPQctrl(struct PQctrl *pPQCvar,
             double *Idq0_inst,
             double *PQP0inst,
             double VDCinst,
             double *PQ_ref,
             double *beta_dq_h1_seqp_ref,
             double *Idq_error){

    // generate current reference from slow power loops
    double Idq_ref[2];
    Idq_ref[0] =  1.0 * stepPI(&(pPQCvar->activePI)  , PQ_ref[0] - PQP0inst[0]);
    Idq_ref[1] = -1.0 * stepPI(&(pPQCvar->reactivePI), PQ_ref[1] - PQP0inst[1]);

    Idq_error[0] = Idq_ref[0] - Idq0_inst[0];
    Idq_error[1] = Idq_ref[1] - Idq0_inst[1];

    stepDQctrl(&(pPQCvar->DQCvar), Idq0_inst, VDCinst, Idq_ref, beta_dq_h1_seqp_ref);


    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct PreloadCtrl{

    // DC Link Preload process controller

    int thresh;              // Voltage threshold
    int Vdc_stack;             // Reference Value
    int minVdc;              // Absolute minimum DC voltage
    bool withinThresh;       // State variable
    bool converged;          // State state variable
};

void initPLctrl(struct PreloadCtrl *PLvar, int Vdc_stack, int thresh, int minVdc){

    (*PLvar).thresh = thresh;
    (*PLvar).Vdc_stack = Vdc_stack;
    (*PLvar).minVdc = minVdc;
    (*PLvar).withinThresh = false;
    (*PLvar).converged = false;

}

void resetPLctrl(struct PreloadCtrl *PLvar){

    (*PLvar).withinThresh = false;
    (*PLvar).converged = false;
}

bool stepPLctrl(struct PreloadCtrl *PLvar, double Vdc){

    // update state

    // check thresh
    PLvar->withinThresh = (abs(Vdc - PLvar->Vdc_stack) < PLvar->thresh);

    // update convergence state
    PLvar->converged = false;
    if ((Vdc > PLvar->minVdc) && (PLvar->withinThresh)){
        PLvar->converged = true;
    }

    return PLvar->converged;
}



////////////////////////////////////////////////////////////////////////////////////////////////


void measurementConversion(uint16_t *Vabc_adc, uint16_t *Iabc_adc, uint16_t Vdc_adc,
                           double *gVACcalibration, double *gIACcalibration,
                           double *Vabc, double *Iabc, double *Vdc){

    int i;
    for(i = 0; i < 3; i++){
        Vabc[i] = VAC_ANALOG_GAIN_SIGN * (( (double)Vabc_adc[i]) - gVACcalibration[i]) * VAC_SCALE;
        Iabc[i] = IAC_ANALOG_GAIN_SIGN * (( (double)Iabc_adc[i]) - gIACcalibration[i]) * IAC_SCALE / ((double) TURNS_NUMBER);
    }
    *Vdc = ((double) Vdc_adc - VDC_BASE) * VDC_SCALE;
}

////////////////////////////////////////////////////////////////////////////////////////////////

struct ADC_calibration{

    struct LPF1 VaLPF;
    struct LPF1 VbLPF;
    struct LPF1 VcLPF;

    struct LPF1 IaLPF;
    struct LPF1 IbLPF;
    struct LPF1 IcLPF;

};

void initADCcalibration(struct ADC_calibration* ADCcalVar, double Ts, double cutoff_freq, double initialOut){

    initLPF1(&(ADCcalVar->VaLPF), Ts, cutoff_freq, initialOut);
    initLPF1(&(ADCcalVar->VbLPF), Ts, cutoff_freq, initialOut);
    initLPF1(&(ADCcalVar->VcLPF), Ts, cutoff_freq, initialOut);

    initLPF1(&(ADCcalVar->IaLPF), Ts, cutoff_freq, initialOut);
    initLPF1(&(ADCcalVar->IbLPF), Ts, cutoff_freq, initialOut);
    initLPF1(&(ADCcalVar->IcLPF), Ts, cutoff_freq, initialOut);

}

void stepADCcalibration(struct ADC_calibration* ADCcalVar, uint16_t *Vabc_adc, uint16_t *Iabc_adc){

    stepLPF1(&(ADCcalVar->VaLPF), Vabc_adc[0]);
    stepLPF1(&(ADCcalVar->VbLPF), Vabc_adc[1]);
    stepLPF1(&(ADCcalVar->VcLPF), Vabc_adc[2]);

    stepLPF1(&(ADCcalVar->IaLPF), Vabc_adc[0]);
    stepLPF1(&(ADCcalVar->IbLPF), Vabc_adc[1]);
    stepLPF1(&(ADCcalVar->IcLPF), Vabc_adc[2]);
}

void resetADCcalibration(struct ADC_calibration* ADCcalVar, double initialOut){

    resetLPF1(&(ADCcalVar->VaLPF), initialOut);
    resetLPF1(&(ADCcalVar->VbLPF), initialOut);
    resetLPF1(&(ADCcalVar->VcLPF), initialOut);

    resetLPF1(&(ADCcalVar->IaLPF), initialOut);
    resetLPF1(&(ADCcalVar->IbLPF), initialOut);
    resetLPF1(&(ADCcalVar->IcLPF), initialOut);
}

////////////////////////////////////////////////////////////////////////////////////////////////


void dutyCycleConversion(double *duty_abc, uint16_t *counterValues_abc){
//    HARDCODE FEO
    int i;
    for(i = 0; i < 3; i++){
        duty_abc[i] = - duty_abc[i];
    }

    for(i = 0; i < 3; i++){
        counterValues_abc[i] = (uint16_t)(PRD_PWM_OUT * (duty_abc[i] + 1.0) / 2.0);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////







