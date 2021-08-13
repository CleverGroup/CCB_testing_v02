////////////////////////////////////////////////////////////////////////////////////////////////
// FUNDAMENTALES

// Constantes
#define PI   3.141592653589793238462643383279502884197169399
#define SQ2  1.414213562373095048801688724209698078569671875
#define SQ3  1.732050807568877293527446341505872366942805254

#define _2PI     (2.0*PI)
#define SQ6      (SQ2*SQ3)
#define iSQ2     (SQ2/2.0)
#define iSQ3     (SQ3/3.0)
#define iSQ6     (SQ6/6.0)
#define SQ2_SQ3  (SQ2/SQ3)

////////////////////////////////////////////////////////////////////////////////////////////////
// ELËCTRICOS

// Nominales
#define V_FN_RMS_NOM      (70.0 / SQ3)  // V
#define FG_NOM            50.0          // Hz
#define VDC_NOM           150.0         // V
#define FILTER_INDUCTANCE (2 * 1.5e-3)  // H
#define FILTER_RESISTANCE 0.1           // Ohm
#define TFMER_REACTANCE   0.0           // % base propia
#define TFMER_RESISTANCE  0.0           // Ohm
#define NOM_APP_PWR       (100e3)       // VA
#define DCLINK_CAP        (1.4e-3)      // F
#define PRELOAD_R         (4.0*470.0)   // Ohm

// Derivados
#define WG_NOM              (2.0 * PI * FG_NOM)                     // rad/s
#define V_FF_RMS_NOM        (V_FN_RMS_NOM * SQ3)                    // V
#define TFMER_INDUCTANCE    ((pow(V_FF_RMS_NOM, 2)/NOM_APP_PWR) * (TFMER_REACTANCE/100.0) / WG_NOM) // H
#define TOTAL_INDUCTANCE    (TFMER_INDUCTANCE + FILTER_INDUCTANCE)  // H
#define TOTAL_RESISTANCE    (TFMER_RESISTANCE + FILTER_RESISTANCE)  // Ohm
#define OL_CURRENT_TIMECONS (TOTAL_INDUCTANCE / TOTAL_RESISTANCE)   // s/rad
#define OL_CURRENT_GAIN     (VDC_NOM / (2 * TOTAL_RESISTANCE))      // A
#define NOMINAL_REACTANCE   (WG_NOM*TOTAL_INDUCTANCE)               // Ohm


////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL

// MCU
#define MCclock (unsigned long) (200e6)  // MC clock register

// Generales
#define FS (10e3)       // sampling frequency, Hz
#define TS (1.0 / FS)   // sampling period, s

// PWM
#define FREQ_MOD_IDX_SEED   16                                          // 1, Whole number to select only optimal F_switching. 10 = 3150Hz, 16 = 4950Hz.
#define DEAD_TIME           (500e-9)                                    // s, Inverter switching dead time

#define FREQ_MOD_IDX        (6 * FREQ_MOD_IDX_SEED + 3)                 // 1, ODD multiple of THREE
#define F_SWITCHING         (FREQ_MOD_IDX * FG_NOM)                     // Hz, Switching frequency

#define PRD_PWM_OUT     (uint16_t)(MCclock / (2.0 * 2.0 * F_SWITCHING)) // PRD register value for output PWM's
#define PRD_PWM_TRIGGER (uint16_t)(MCclock / (2.0 * 2.0 * FS))          // PRD register value for Control trigger PWM's

// Power Analyser
#define PWR_LPF_CUTOFF          10.0    // Hz
#define PWR_LPF_ORDER           3       // 1

// Grid Analyser
#define SOGIGAIN                4.0

#define KP_PI_PLL               180.0
#define KI_PI_PLL               3200.0

#define PLL_ERROR_LPF_TC        0.08    // s
#define PLL_ERROR_LPF_INIT      1.0     // 1
#define PLL_ERROR_THRESH        0.03    // 1

#define PLL_QAXIS_NOTCH_NUMBER  6       // 1
#define PLL_QAXIS_NOTCH_RHO     0.999   // 1

#define PLL_OMEGA_LPF_ORDER     2       // 1
#define PLL_OMEGA_LPF_FREQ      10.0    // Hz

// DC voltage measurement filter
#define VDC_LPF_CUTOFF          10.0    // Hz

// DQ current control specifications
#define CL_CURRENT_DAMPING      0.85    // 1, Closed-Loop current damping specification
#define CL_CURRENT_TIMECONS     (2e-3)  // s, Closed-loop settling time


// DQ current control derived
#define TI_PI_CURRENT  (2.0 * CL_CURRENT_DAMPING * CL_CURRENT_TIMECONS - pow(CL_CURRENT_TIMECONS, 2) / OL_CURRENT_TIMECONS) // Integral time
#define KP_PI_CURRENT  (OL_CURRENT_TIMECONS * TI_PI_CURRENT / (OL_CURRENT_GAIN * pow(CL_CURRENT_TIMECONS, 2)))              // Proportional Gain
#define KI_PI_CURRENT  (KP_PI_CURRENT / TI_PI_CURRENT)                                                                      // Integral Gain

#define BETA_D_EQ (V_FN_RMS_NOM * SQ2 / (VDC_NOM / 2.0))        // nominal equilibrium duty cycle


// DC preload params
#define VDC_PRELOAD_THRESH  5                                   // V, maximum admissible difference between Vdc and reference
#define VDC_MIN             (V_FN_RMS_NOM * SQ2 * SQ3 * 1.1)    // V, minimum admissible DC voltage overall


// POWER control
//TI_PI controls response time, keep KP_PI low to avoid sharp current reference changes
#define TI_PI_P 100.00              // W.s/A
#define KP_PI_P (0.05e-3)           // A/W
#define KI_PI_P (1/TI_PI_P)         // A/W.s

#define TI_PI_Q 100.00              // VAr.s/A
#define KP_PI_Q (0.05e-3)           // A/VAr
#define KI_PI_Q (1/TI_PI_Q)         // A/VAr.s

//Reference rate-of-change limit (slew rate)
#define SR_P (NOM_APP_PWR / 30.0)   // W/s
#define SR_Q (NOM_APP_PWR / 60.0)   // V.A/s


// VOLTAGE CONTROL for STATCOM mode
#define TI_PI_V 1.00                // V.s/A
#define KP_PI_V 0.05                // A/V
#define KI_PI_V (1/TI_PI_V)         // A/V.s


// Higher Harmonic Compensation
#define HARMONIC1   6               // Compensation is made to HARMONIC1 + 1 and HARMONIC1 - 1
#define KP_PR_H1    0.0             // V/A
#define KR_PR_H1    KI_PI_CURRENT   //

#define HARMONIC2   12              // Compensation is made to HARMONIC2 + 1 and HARMONIC2 - 1
#define KP_PR_H2    0.0             // V/A
#define KR_PR_H2    KI_PI_CURRENT   //



////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORS

//Parámetros Calibración Sensores
#define VAC_BASE                2037.3333333    // ticks
#define VAC_SCALE               0.205049801     // V/tick
#define VAC_ANALOG_GAIN_SIGN    -1.0            // sign of analog stage gain

#define IAC_BASE                2042.66666666   // ticks
#define IAC_SCALE               0.117216117216  // A/tick with 1 turn
#define IAC_ANALOG_GAIN_SIGN    -1.0            // sign of analog stage gain
#define TURNS_NUMBER            1               // integer number of turns around sensor ferrite core

#define VDC_BASE                55.122331       // ticks
#define VDC_SCALE               0.2429922       // V/tick

#define OFFSET_CALIBRATION_LPF_FREQ     1.0     // Hz
#define OFFSET_CALIBRATION_LPF_INIT     2047.0  // ticks, 2^ADC_RESOLUTION - 1. Assuming CCB potentiometers calibrated


////////////////////////////////////////////////////////////////////////////////////////////////
// COMMUNICATION

#define MAX_SEND_CAN 3

////////////////////////////////////////////////////////////////////////////////////////////////


// Funciones Básicas
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#define SAT(X) MIN(1, MAX(-1, X))
#define SIGN(X) (X > 0) ? 1 : ((X < 0) ? -1 : 0)

