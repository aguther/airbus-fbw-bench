#ifndef RTW_HEADER_FlyByWire_h_
#define RTW_HEADER_FlyByWire_h_
#include <cmath>
#include <cstring>
#ifndef FlyByWire_COMMON_INCLUDES_
# define FlyByWire_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "FlyByWire_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

typedef struct {
  real_T DiscreteTransferFcn1_states;
  real_T UD_DSTATE;
  real_T Integration_DSTATE;
  real_T Integration13s_DSTATE;
  real_T DiscreteTimeIntegrator_DSTATE;
  real_T PrevY;
  real_T PrevY_c;
  real_T PrevY_h;
  real_T PrevY_j;
  real_T PrevY_i[3];
  real_T PrevY_k;
  real_T PrevY_jv;
  real_T PrevY_d;
  real_T PrevY_jg;
  int32_T chartAbsoluteTimeCounter;
  int32_T durationLastReferenceTick_1;
  int8_T Integration_PrevResetState;
  int8_T Integration13s_PrevResetState;
  int8_T DiscreteTimeIntegrator_PrevResetState;
  uint8_T Integration_IC_LOADING;
  uint8_T DiscreteTimeIntegrator_IC_LOADING;
  uint8_T is_active_c5_FlyByWire;
  uint8_T is_c5_FlyByWire;
  uint8_T is_active_c3_FlyByWire;
  uint8_T is_c3_FlyByWire;
  uint8_T is_active_c1_FlyByWire;
  uint8_T is_c1_FlyByWire;
  boolean_T condWasTrueAtLastTimeStep_1;
} D_Work_FlyByWire_T;

typedef struct {
  fbw_input in;
} ExternalInputs_FlyByWire_T;

typedef struct {
  fbw_output out;
} ExternalOutputs_FlyByWire_T;

struct Parameters_FlyByWire_T_ {
  fbw_output fbw_output_MATLABStruct;
  real_T PID_D;
  real_T PID_DifferentiatorICPrevScaledInput;
  real_T PID_LowerSaturationLimit;
  real_T PID_P;
  real_T PID_UpperSaturationLimit;
  real_T Constant_Value;
  real_T GainTheta_Gain;
  real_T GainPhi_Gain;
  real_T Gain_Gain;
  real_T Gainqk_Gain;
  real_T Gain_Gain_l;
  real_T Gainrk_Gain;
  real_T Gain_Gain_a;
  real_T Gainpk_Gain;
  real_T RateLimitereta_RisingLim;
  real_T RateLimitereta_FallingLim;
  real_T RateLimitereta_IC;
  real_T Gaineta_Gain;
  real_T RateLimiterxi_RisingLim;
  real_T RateLimiterxi_FallingLim;
  real_T RateLimiterxi_IC;
  real_T Gainxi_Gain;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T RateLimit_RisingLim;
  real_T RateLimit_FallingLim;
  real_T RateLimit_IC;
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
  real_T Gain1_Gain;
  real_T Constant2_Value;
  real_T Gain1_Gain_p;
  real_T Gain1_Gain_b;
  real_T RateLimiter_RisingLim;
  real_T RateLimiter_FallingLim;
  real_T RateLimiter_IC;
  real_T loaddemand_tableData[3];
  real_T loaddemand_bp01Data[3];
  real_T Saturation_UpperSat_e;
  real_T Saturation_LowerSat_n;
  real_T TSamp_WtEt;
  real_T Limiter_RisingLim;
  real_T Limiter_FallingLim;
  real_T Limiter_IC;
  real_T Saturation_UpperSat_h;
  real_T Saturation_LowerSat_l;
  real_T Integration_gainval;
  real_T Integration_UpperSat;
  real_T Integration_LowerSat;
  real_T Constant_Value_i;
  real_T Integration13s_gainval;
  real_T Integration13s_IC;
  real_T Integration13s_UpperSat;
  real_T Integration13s_LowerSat;
  real_T RateLimiter03s_RisingLim;
  real_T RateLimiter03s_FallingLim;
  real_T RateLimiter03s_IC;
  real_T Saturation_UpperSat_p;
  real_T Saturation_LowerSat_h;
  real_T RateLimit_RisingLim_b;
  real_T RateLimit_FallingLim_m;
  real_T RateLimit_IC_d;
  real_T Gain1_Gain_m;
  real_T BankAngleProtection_tableData[7];
  real_T BankAngleProtection_bp01Data[7];
  real_T RateLimiter_UpperSat;
  real_T RateLimiter_LowerSat;
  real_T DiscreteTimeIntegrator_gainval;
  real_T DiscreteTimeIntegrator_UpperSat;
  real_T DiscreteTimeIntegrator_LowerSat;
  real_T Gain2_Gain;
  real_T Gain1_Gain_mg;
  real_T pKp_Gain;
  real_T Constant_Value_d;
  real_T RateLimitereta_RisingLim_j;
  real_T RateLimitereta_FallingLim_m;
  real_T RateLimitereta_IC_c;
  real_T Gaineta_Gain_d;
  real_T Limitereta_UpperSat;
  real_T Limitereta_LowerSat;
  real_T GainiH_Gain;
  real_T LimiteriH_UpperSat;
  real_T LimiteriH_LowerSat;
  real_T RateLimiterxi_RisingLim_n;
  real_T RateLimiterxi_FallingLim_f;
  real_T RateLimiterxi_IC_c;
  real_T Gainxi_Gain_n;
  real_T Limiterxi_UpperSat;
  real_T Limiterxi_LowerSat;
  real_T Convertto_Gain;
};

extern const fbw_input FlyByWire_rtZfbw_input;
extern const fbw_output FlyByWire_rtZfbw_output;
class FlyByWireModelClass {
 public:
  ExternalInputs_FlyByWire_T FlyByWire_U;
  ExternalOutputs_FlyByWire_T FlyByWire_Y;
  void initialize();
  void step();
  void terminate();
  FlyByWireModelClass();
  ~FlyByWireModelClass();
 private:
  static Parameters_FlyByWire_T FlyByWire_P;
  D_Work_FlyByWire_T FlyByWire_DWork;
};

#endif

