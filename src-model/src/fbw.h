#ifndef RTW_HEADER_fbw_h_
#define RTW_HEADER_fbw_h_
#include <cmath>
#include <cstring>
#ifndef fbw_COMMON_INCLUDES_
# define fbw_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "fbw_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real_T UD_DSTATE;
  real_T DiscreteTransferFcn1_states;
  real_T Integration_DSTATE;
  real_T Integration13s_DSTATE;
  real_T DiscreteTimeIntegrator_DSTATE;
  real_T PrevY;
  real_T PrevY_e;
  real_T PrevY_g[3];
  real_T PrevY_p;
  real_T PrevY_d;
  real_T PrevY_n;
  real_T PrevY_gj;
  real_T PrevY_c;
  real_T PrevY_j;
  int32_T chartAbsoluteTimeCounter;
  int32_T durationLastReferenceTick_1;
  int8_T Integration_PrevResetState;
  int8_T Integration13s_PrevResetState;
  int8_T DiscreteTimeIntegrator_PrevResetState;
  uint8_T Integration_IC_LOADING;
  uint8_T DiscreteTimeIntegrator_IC_LOADING;
  uint8_T is_active_c5_fbw;
  uint8_T is_c5_fbw;
  uint8_T is_active_c3_fbw;
  uint8_T is_c3_fbw;
  uint8_T is_active_c1_fbw;
  uint8_T is_c1_fbw;
  boolean_T condWasTrueAtLastTimeStep_1;
} D_Work_fbw_T;

typedef struct {
  real_T in_sim_simrawdata_nz_g;
  real_T in_sim_simrawdata_Theta_deg;
  real_T in_sim_simrawdata_Phi_deg;
  real_T in_sim_simrawdata_qk_rad_s;
  real_T in_sim_simrawdata_rk_rad_s;
  real_T in_sim_simrawdata_pk_rad_s;
  real_T in_sim_simrawdata_Vk_kt;
  real_T in_sim_simrawdata_radio_height_ft;
  real_T in_sim_simrawdata_CG_percent_MAC;
  real_T in_sim_simrawinput_delta_eta_pos;
  real_T in_sim_simrawinput_delta_xi_pos;
} ExternalInputs_fbw_T;

typedef struct {
  real_T out_sim_simrawdata_nz_g;
  real_T out_sim_simrawdata_Theta_deg;
  real_T out_sim_simrawdata_Phi_deg;
  real_T out_sim_simrawdata_qk_rad_s;
  real_T out_sim_simrawdata_rk_rad_s;
  real_T out_sim_simrawdata_pk_rad_s;
  real_T out_sim_simrawdata_Vk_kt;
  real_T out_sim_simrawdata_radio_height_ft;
  real_T out_sim_simrawdata_CG_percent_MAC;
  real_T out_sim_simrawinput_delta_eta_pos;
  real_T out_sim_simrawinput_delta_xi_pos;
  real_T out_sim_simdata_nz_g;
  real_T out_sim_simdata_Theta_deg;
  real_T out_sim_simdata_Phi_deg;
  real_T out_sim_simdata_qk_deg_s;
  real_T out_sim_simdata_rk_deg_s;
  real_T out_sim_simdata_pk_deg_s;
  real_T out_sim_simdata_Vk_kt;
  real_T out_sim_simdata_radio_height_ft;
  real_T out_sim_simdata_CG_percent_MAC;
  real_T out_sim_simdatacomputed_on_ground;
  real_T out_sim_siminput_delta_eta_pos;
  real_T out_sim_siminput_delta_xi_pos;
  real_T out_sim_simrawoutput_eta_pos;
  real_T out_sim_simrawoutput_iH_deg;
  real_T out_sim_simrawoutput_xi_pos;
  real_T out_pitch_pitchdatacomputed_in_flight;
  real_T out_pitch_pitchdatacomputed_in_flare;
  real_T out_pitch_pitchdatacomputed_in_flight_gain;
  real_T out_pitch_pitchnormal_Cstar_g;
  real_T out_pitch_pitchnormal_Cstar_c_g;
  real_T out_pitch_pitchnormal_eta_dot_pos_s;
  real_T out_pitch_pitchprotectionattitudemin_eta_dot_pos_s;
  real_T out_pitch_pitchprotectionattitudemax_eta_dot_pos_s;
  real_T out_pitch_pitchvote_eta_dot_pos_s;
  real_T out_pitch_pitchintegrated_eta_pos;
  real_T out_pitch_pitchoutput_eta_pos;
  real_T out_pitch_pitchoutput_iH_deg;
  real_T out_roll_rolldatacomputed_in_flight;
  real_T out_roll_rolldatacomputed_in_flight_gain;
  real_T out_roll_rollnormal_pk_c_deg_s;
  real_T out_roll_rollnormal_Phi_c_deg;
  real_T out_roll_rollnormal_xi_pos;
  real_T out_roll_rolloutput_xi_pos;
} ExternalOutputs_fbw_T;

struct Parameters_fbw_T_ {
  real_T PID_D;
  real_T PID_DifferentiatorICPrevScaledInput;
  real_T PID_LowerSaturationLimit;
  real_T PID_P;
  real_T PID_UpperSaturationLimit;
  real_T Constant_Value;
  real_T Constant2_Value;
  real_T Gain_Gain;
  real_T Gainqk_Gain;
  real_T Gain1_Gain;
  real_T GainTheta_Gain;
  real_T Gain1_Gain_c;
  real_T GainPhi_Gain;
  real_T Gain1_Gain_ci;
  real_T RateLimitereta_RisingLim;
  real_T RateLimitereta_FallingLim;
  real_T RateLimitereta_IC;
  real_T Gaineta_Gain;
  real_T RateLimiter_RisingLim;
  real_T RateLimiter_FallingLim;
  real_T RateLimiter_IC;
  real_T loaddemand_tableData[3];
  real_T loaddemand_bp01Data[3];
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T TSamp_WtEt;
  real_T DiscreteTransferFcn1_NumCoef[2];
  real_T DiscreteTransferFcn1_DenCoef[2];
  real_T DiscreteTransferFcn1_InitialStates;
  real_T K7857_Gain;
  real_T K7958_Gain;
  real_T u1_Value;
  real_T K7659_Gain;
  real_T F193LUT_tableData[8];
  real_T F193LUT_bp01Data[8];
  real_T u7_Value;
  real_T K7765_Gain;
  real_T Limiter_RisingLim;
  real_T Limiter_FallingLim;
  real_T Limiter_IC;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_d;
  real_T Integration_gainval;
  real_T Integration_UpperSat;
  real_T Integration_LowerSat;
  real_T Saturation_UpperSat_ks;
  real_T Saturation_LowerSat_k;
  real_T RateLimit_RisingLim;
  real_T RateLimit_FallingLim;
  real_T RateLimit_IC;
  real_T Constant_Value_n;
  real_T Convertto_Gain;
  real_T RateLimitereta_RisingLim_j;
  real_T RateLimitereta_FallingLim_m;
  real_T RateLimitereta_IC_c;
  real_T Gaineta_Gain_d;
  real_T Limitereta_UpperSat;
  real_T Limitereta_LowerSat;
  real_T Integration13s_gainval;
  real_T Integration13s_IC;
  real_T Integration13s_UpperSat;
  real_T Integration13s_LowerSat;
  real_T RateLimiter03s_RisingLim;
  real_T RateLimiter03s_FallingLim;
  real_T RateLimiter03s_IC;
  real_T GainiH_Gain;
  real_T LimiteriH_UpperSat;
  real_T LimiteriH_LowerSat;
  real_T Saturation_UpperSat_f;
  real_T Saturation_LowerSat_n;
  real_T RateLimit_RisingLim_e;
  real_T RateLimit_FallingLim_i;
  real_T RateLimit_IC_b;
  real_T Constant_Value_b;
  real_T RateLimiterxi_RisingLim;
  real_T RateLimiterxi_FallingLim;
  real_T RateLimiterxi_IC;
  real_T Gainxi_Gain;
  real_T DiscreteTimeIntegrator_gainval;
  real_T DiscreteTimeIntegrator_UpperSat;
  real_T DiscreteTimeIntegrator_LowerSat;
  real_T Gain2_Gain;
  real_T Gain_Gain_a;
  real_T Gainpk_Gain;
  real_T Gain1_Gain_g;
  real_T pKp_Gain;
  real_T RateLimiterxi_RisingLim_n;
  real_T RateLimiterxi_FallingLim_f;
  real_T RateLimiterxi_IC_c;
  real_T Gainxi_Gain_n;
  real_T Limiterxi_UpperSat;
  real_T Limiterxi_LowerSat;
  real_T Gain1_Gain_h;
  real_T BankAngleProtection_tableData[7];
  real_T BankAngleProtection_bp01Data[7];
  real_T RateLimiter_UpperSat;
  real_T RateLimiter_LowerSat;
  real_T Gain_Gain_l;
  real_T Gainrk_Gain;
};

struct tag_RTM_fbw_T {
  const char_T * volatile errorStatus;
};

class fbwModelClass {
 public:
  ExternalInputs_fbw_T fbw_U;
  ExternalOutputs_fbw_T fbw_Y;
  void initialize();
  void step();
  void terminate();
  fbwModelClass();
  ~fbwModelClass();
  RT_MODEL_fbw_T * getRTM();
 private:
  static Parameters_fbw_T fbw_P;
  D_Work_fbw_T fbw_DWork;
  RT_MODEL_fbw_T fbw_M;
};

#endif

