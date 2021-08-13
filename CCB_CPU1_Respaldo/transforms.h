///////////// CLARKE TRANSFORMS

void fwdClarkePI(double *abc, double *abg_results){

    abg_results[0] = 2.0 *  abc[0] -       (abc[1] + abc[2]);
    abg_results[1] =               + SQ3 * (abc[1] - abc[2]);
    abg_results[2] = SQ2 * (abc[0] +        abc[1] + abc[2]);

	int i;
    for(i = 0; i < 3; i++){
        abg_results[i] *= iSQ6;
    }

    return;
}

void revClarkePI(double *abg, double *abc_results){

    double sq3beta  =  SQ3 * abg[1];
    double sq2gamma =  SQ2 * abg[2];

    abc_results[0] = + 2.0 * abg[0]           + sq2gamma;
    abc_results[1] = -       abg[0] + sq3beta + sq2gamma;
    abc_results[2] = -       abg[0] - sq3beta + sq2gamma;

    int i;
    for(i = 0; i < 3; i++){
        abc_results[i] *= iSQ6;
    }

    return;
}



void fwdClarkePV(double *abc, double *abg_results){

    abg_results[0] = 2.0 *  abc[0] -       (abc[1] + abc[2]);
    abg_results[1] =               + SQ3 * (abc[1] - abc[2]);
    abg_results[2] =        abc[0] +        abc[1] + abc[2];

	int i;
	double third = 1.0/3.0;
    for(i = 0; i < 3; i++){
        abg_results[i] *= third;
    }

    return;
}

void revClarkePV(double *abg, double *abc_results){

    double sq3beta =  SQ3 * abg[1];
    double _2gamma =  2.0 * abg[2];

    abc_results[0] = + 2.0 * abg[0]           + _2gamma;
    abc_results[1] = -       abg[0] + sq3beta + _2gamma;
    abc_results[2] = -       abg[0] - sq3beta + _2gamma;

    double half = 0.5;
    int i;
    for(i = 0; i < 3; i++){
        abc_results[i] *= half;
    }

    return;
}



///////////// PARK TRANSFORMS

void fwdParkQ(double *abg, double theta, double *dq0_results){
//    IMPORTANT: a-phase aligned with q-axis

    double s = sin(theta);
    double c = cos(theta);

    dq0_results[0] = s * abg[0] - c * abg[1];
    dq0_results[1] = c * abg[0] + s * abg[1];
    dq0_results[2] =                  abg[2];

    return;
}

void revParkQ(double *dq0, double theta, double *abg_results){
//    IMPORTANT: a-phase aligned with q-axis

    double s = sin(theta);
    double c = cos(theta);

    abg_results[0] =   s * dq0[0] + c * dq0[1];
    abg_results[1] = - c * dq0[0] + s * dq0[1];
    abg_results[2] =                    dq0[2];

    return;
}



void fwdParkD(double *abg, double theta, double *dq0_results){
//    IMPORTANT: a-phase aligned with d-axis

    double s = sin(theta);
    double c = cos(theta);

    dq0_results[0] =   c * abg[0] + s * abg[1];
    dq0_results[1] = - s * abg[0] + c * abg[1];
    dq0_results[2] =                    abg[2];

    return;
}

void revParkD(double *dq0, double theta, double *abg_results){
//    IMPORTANT: a-phase aligned with d-axis

    double s = sin(theta);
    double c = cos(theta);

    abg_results[0] = c * dq0[0] - s * dq0[1];
    abg_results[1] = s * dq0[0] + c * dq0[1];
    abg_results[2] =                  dq0[2];

    return;
}

