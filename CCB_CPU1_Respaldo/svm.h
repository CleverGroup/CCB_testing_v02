

void SVM(double *AlphaBetaGamma, double *uvw_results){

    // Carrier Based SVMPWM

    revClarkePV(AlphaBetaGamma, uvw_results);

    double _max = MAX(MAX(uvw_results[0], uvw_results[1]), uvw_results[2]);
    double _min = MIN(MIN(uvw_results[0], uvw_results[1]), uvw_results[2]);

    double common = (_max + _min)/2.0;

    uvw_results[0] = SAT(uvw_results[0] - common);
    uvw_results[1] = SAT(uvw_results[1] - common);
    uvw_results[2] = SAT(uvw_results[2] - common);

    return;
}

