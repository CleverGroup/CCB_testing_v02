#include <CleverControl.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////////////////////

struct LPFN{

    // Nth order IIR low pass filter

    uint8_t order;
    double out;

    struct LPFN* filters;

};


void initLPFN(struct LPFN *pLPFNvar, uint8_t order, double Ts, double cutoff_freq, double initialOut){

    pLPFNvar->order = order;
    pLPFNvar->out = initialOut;

    pLPFNvar->filters = malloc(order * sizeof(struct LPF1));

    int i;
    for(i = 0; i < pLPFNvar->order; i++){
        initLPF1(&pLPFNvar->filters[i], Ts, cutoff_freq, initialOut);
    }
}

void resetLPFN(struct LPFN *pLPFNvar, double initialOut){

    pLPFNvar->out = initialOut;

    int i;
    for(i = 0; i < pLPFNvar->order; i++){
        resetLPF1(&pLPFNvar->filters[i], initialOut);
    }
}

double stepLPFN(struct LPFN *pLPFNvar, double input){

    int i;
    double intermediateOutput = input;
    for(i = 0; i < pLPFNvar->order; i++){
        intermediateOutput = stepLPF1(&pLPFNvar->filters[i], intermediateOutput);
    }

    pLPFNvar->out = intermediateOutput;

    return pLPFNvar->out;
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
        initNotch(&pNchNvar->filters[i], Ts, rho, initialState);
    }
}

void resetNotchN(struct NotchN *pNchNvar, double initialState){

    int i;
    for(i = 0; i < pNchNvar->order; i++){
        resetNotch(&pNchNvar->filters[i], initialState);
    }
}

double stepNotchN(struct NotchN *pNchNvar, double input, double w0){

    double output = input;

    int i;
    for(i = 0; i < pNchNvar->order; i++){
        output = stepNotch(&pNchNvar->filters[i], output, w0*(i+1));
    }

    pNchNvar->out = output;

    return output;
}







