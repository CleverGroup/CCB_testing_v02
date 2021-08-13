////////////////////////////////////////////////////////////////////////////////////////////////

struct StateVariableMonitor{

    double PQp0_seqp_inst[3];
    double PQ_seqp_ref[2];

    double Iabc_inst[3];
    double Vabc_inst[3];

    double Idq0_inst[3];
    double Idq0_ref[3];

    double beta_dq0[3];
    double beta_abc[3];

    double Vdc;
    double Vdc_filtered;

};
