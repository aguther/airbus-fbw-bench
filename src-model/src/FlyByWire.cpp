#include "FlyByWire.h"
#include "FlyByWire_private.h"

const uint8_T FlyByWire_IN_InAir = 1U;
const uint8_T FlyByWire_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T FlyByWire_IN_OnGround = 2U;
const uint8_T FlyByWire_IN_Flare = 1U;
const uint8_T FlyByWire_IN_Flight_High = 2U;
const uint8_T FlyByWire_IN_Flight_Low = 3U;
const uint8_T FlyByWire_IN_Ground = 4U;
const uint8_T FlyByWire_IN_FlightMode = 1U;
const uint8_T FlyByWire_IN_GroundMode = 2U;
const fbw_output FlyByWire_rtZfbw_output = {
  {
    {
      {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
      },

      {
        0.0,
        0.0
      },

      {
        0.0,
        0.0,
        0.0
      }
    },

    {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0
    },

    {
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    },

    {
      {
        0.0
      },

      {
        0.0
      }
    },

    {
      0.0
    },

    {
      0.0
    },

    {
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    },

    {
      0.0
    }
  }
} ;

const fbw_input FlyByWire_rtZfbw_input = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0 } };

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[], uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

void FlyByWireModelClass::step()
{
  real_T rateLimiterRate;
  real_T denAccum;
  boolean_T condIsTrue;
  real_T rtb_Integration13s;
  real_T rtb_DiscreteTimeIntegrator;
  real_T rtb_Integration;
  real_T rtb_Product;
  real_T rtb_Limitereta;
  real_T rtb_LimiteriH;
  real_T rtb_BusAssignment_l_pitch_law_protection_attitude_min_eta_dot_pos_s;
  real_T rtb_BusAssignment_l_pitch_law_protection_attitude_max_eta_dot_pos_s;
  real_T rtb_BusAssignment_l_roll_output_xi_pos;
  real_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_pk_deg_s;
  real_T rtb_Gaineta;
  real_T rtb_Gainxi;
  int32_T rtb_onGround;
  int32_T rtb_inFlight_e;
  real_T rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  real_T rtb_ProportionalGain;
  real_T rtb_TSamp;
  real_T rtb_BusAssignment_mv_pitch_output_iH_deg;
  real_T rtb_Saturation[3];
  int32_T rtb_inFlight;
  boolean_T tmp;
  boolean_T tmp_0;
  boolean_T tmp_1;
  boolean_T tmp_2;
  int32_T exitg1;
  rtb_LimiteriH = FlyByWire_P.GainTheta_Gain * FlyByWire_U.in.data.Theta_deg;
  rtb_Limitereta = FlyByWire_P.GainPhi_Gain * FlyByWire_U.in.data.Phi_deg;
  rtb_DiscreteTimeIntegrator = FlyByWire_P.Gain_Gain * FlyByWire_U.in.data.qk_rad_s * FlyByWire_P.Gainqk_Gain;
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_pk_deg_s = FlyByWire_P.Gain_Gain_a *
    FlyByWire_U.in.data.pk_rad_s * FlyByWire_P.Gainpk_Gain;
  rateLimiterRate = FlyByWire_U.in.input.delta_eta_pos - FlyByWire_DWork.PrevY;
  if (rateLimiterRate > FlyByWire_P.RateLimitereta_RisingLim) {
    rtb_Integration = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimitereta_FallingLim) {
    rtb_Integration = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_FallingLim;
  } else {
    rtb_Integration = FlyByWire_U.in.input.delta_eta_pos;
  }

  FlyByWire_DWork.PrevY = rtb_Integration;
  rtb_Gaineta = FlyByWire_P.Gaineta_Gain * rtb_Integration;
  rateLimiterRate = FlyByWire_U.in.input.delta_xi_pos - FlyByWire_DWork.PrevY_c;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi_RisingLim) {
    rtb_Integration = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi_FallingLim) {
    rtb_Integration = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_FallingLim;
  } else {
    rtb_Integration = FlyByWire_U.in.input.delta_xi_pos;
  }

  FlyByWire_DWork.PrevY_c = rtb_Integration;
  rtb_Gainxi = FlyByWire_P.Gainxi_Gain * rtb_Integration;
  if (FlyByWire_DWork.is_active_c1_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c1_FlyByWire = 1U;
    FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
    rtb_onGround = 1;
  } else if (FlyByWire_DWork.is_c1_FlyByWire == FlyByWire_IN_InAir) {
    if (FlyByWire_U.in.data.radio_height_ft <= 10.0) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
      rtb_onGround = 1;
    } else {
      rtb_onGround = 0;
    }
  } else {
    if (FlyByWire_U.in.data.radio_height_ft > 10.0) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_InAir;
      rtb_onGround = 0;
    } else {
      rtb_onGround = 1;
    }
  }

  FlyByWire_Y.out.sim.data.Phi_deg = rtb_Limitereta;
  FlyByWire_Y.out.sim.data.qk_deg_s = rtb_DiscreteTimeIntegrator;
  FlyByWire_DWork.chartAbsoluteTimeCounter++;
  condIsTrue = ((rtb_onGround == 1) && (rtb_LimiteriH < 2.5));
  if ((!condIsTrue) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
    FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
  }

  FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
  if (FlyByWire_DWork.is_active_c3_FlyByWire == 0U) {
    FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
    FlyByWire_DWork.is_active_c3_FlyByWire = 1U;
    FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground;
    rtb_inFlight_e = 0;
    FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
  } else {
    switch (FlyByWire_DWork.is_c3_FlyByWire) {
     case FlyByWire_IN_Flare:
      condIsTrue = ((rtb_onGround == 1) && (rtb_LimiteriH < 2.5));
      if ((!condIsTrue) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
        FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
      }

      FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
      if (FlyByWire_DWork.chartAbsoluteTimeCounter - FlyByWire_DWork.durationLastReferenceTick_1 > 250) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground;
        rtb_inFlight_e = 0;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      } else if ((FlyByWire_U.in.data.radio_height_ft > 50.0) && (rtb_LimiteriH > 8.0)) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      } else {
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 1.0;
      }
      break;

     case FlyByWire_IN_Flight_High:
      if (FlyByWire_U.in.data.radio_height_ft <= 50.0) {
        FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flare;
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 1.0;
        FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = ((rtb_onGround == 1) && (rtb_LimiteriH < 2.5));
      } else {
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      }
      break;

     case FlyByWire_IN_Flight_Low:
      if (FlyByWire_U.in.data.radio_height_ft > 50.0) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_High;
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      } else {
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      }
      break;

     default:
      if (((rtb_onGround == 0) && (rtb_LimiteriH > 8.0)) || (FlyByWire_U.in.data.radio_height_ft > 400.0)) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_inFlight_e = 1;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      } else {
        rtb_inFlight_e = 0;
        FlyByWire_Y.out.pitch.data_computed.in_flare = 0.0;
      }
      break;
    }
  }

  if (rtb_inFlight_e > FlyByWire_P.Saturation_UpperSat) {
    rtb_Integration = FlyByWire_P.Saturation_UpperSat;
  } else if (rtb_inFlight_e < FlyByWire_P.Saturation_LowerSat) {
    rtb_Integration = FlyByWire_P.Saturation_LowerSat;
  } else {
    rtb_Integration = rtb_inFlight_e;
  }

  rateLimiterRate = rtb_Integration - FlyByWire_DWork.PrevY_h;
  if (rateLimiterRate > FlyByWire_P.RateLimit_RisingLim) {
    rtb_Integration = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_RisingLim;
  } else {
    if (rateLimiterRate < FlyByWire_P.RateLimit_FallingLim) {
      rtb_Integration = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_FallingLim;
    }
  }

  FlyByWire_DWork.PrevY_h = rtb_Integration;
  rtb_BusAssignment_c_pitch_data_computed_in_flight_gain = rtb_Integration;
  denAccum = (rtb_DiscreteTimeIntegrator - FlyByWire_P.DiscreteTransferFcn1_DenCoef[1] *
              FlyByWire_DWork.DiscreteTransferFcn1_states) / FlyByWire_P.DiscreteTransferFcn1_DenCoef[0];
  rtb_Integration = (FlyByWire_P.DiscreteTransferFcn1_NumCoef[0] * denAccum + FlyByWire_P.DiscreteTransferFcn1_NumCoef[1]
                     * FlyByWire_DWork.DiscreteTransferFcn1_states) * FlyByWire_P.K7857_Gain + FlyByWire_P.K7958_Gain *
    rtb_DiscreteTimeIntegrator;
  rtb_Integration13s = look1_binlxpw(FlyByWire_U.in.data.Vk_kt, FlyByWire_P.F193LUT_bp01Data,
    FlyByWire_P.F193LUT_tableData, 7U);
  rtb_BusAssignment_l_pitch_law_protection_attitude_min_eta_dot_pos_s = ((FlyByWire_P.u1_Value + rtb_LimiteriH) *
    FlyByWire_P.K7659_Gain + rtb_Integration) * rtb_Integration13s;
  rtb_BusAssignment_l_pitch_law_protection_attitude_max_eta_dot_pos_s = ((rtb_LimiteriH - FlyByWire_P.u7_Value) *
    FlyByWire_P.K7765_Gain + rtb_Integration) * rtb_Integration13s;
  rtb_Integration = FlyByWire_P.Gain1_Gain * FlyByWire_U.in.data.qk_rad_s * FlyByWire_P.Constant2_Value +
    FlyByWire_U.in.data.nz_g;
  rateLimiterRate = rtb_Gaineta - FlyByWire_DWork.PrevY_j;
  if (rateLimiterRate > FlyByWire_P.RateLimiter_RisingLim) {
    rtb_Integration13s = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiter_FallingLim) {
    rtb_Integration13s = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimiter_FallingLim;
  } else {
    rtb_Integration13s = rtb_Gaineta;
  }

  FlyByWire_DWork.PrevY_j = rtb_Integration13s;
  rtb_Integration13s = look1_binlxpw(rtb_Integration13s, FlyByWire_P.loaddemand_bp01Data,
    FlyByWire_P.loaddemand_tableData, 2U);
  rtb_Integration13s += std::cos(FlyByWire_P.Gain1_Gain_p * rtb_LimiteriH) / std::cos(FlyByWire_P.Gain1_Gain_b *
    rtb_Limitereta);
  if (rtb_Integration13s > FlyByWire_P.Saturation_UpperSat_e) {
    rtb_Integration13s = FlyByWire_P.Saturation_UpperSat_e;
  } else {
    if (rtb_Integration13s < FlyByWire_P.Saturation_LowerSat_n) {
      rtb_Integration13s = FlyByWire_P.Saturation_LowerSat_n;
    }
  }

  rtb_DiscreteTimeIntegrator = rtb_Integration - rtb_Integration13s;
  rtb_ProportionalGain = FlyByWire_P.PID_P * rtb_DiscreteTimeIntegrator;
  rtb_DiscreteTimeIntegrator *= FlyByWire_P.PID_D;
  rtb_TSamp = rtb_DiscreteTimeIntegrator * FlyByWire_P.TSamp_WtEt;
  rtb_DiscreteTimeIntegrator = (rtb_TSamp - FlyByWire_DWork.UD_DSTATE) + rtb_ProportionalGain;
  if (rtb_DiscreteTimeIntegrator > FlyByWire_P.PID_UpperSaturationLimit) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.PID_UpperSaturationLimit;
  } else {
    if (rtb_DiscreteTimeIntegrator < FlyByWire_P.PID_LowerSaturationLimit) {
      rtb_DiscreteTimeIntegrator = FlyByWire_P.PID_LowerSaturationLimit;
    }
  }

  rateLimiterRate = rtb_DiscreteTimeIntegrator - FlyByWire_DWork.PrevY_i[0];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[0] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[0] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[0] = rtb_DiscreteTimeIntegrator;
  }

  if (FlyByWire_DWork.PrevY_i[0] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[0] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[0] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[0] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[0] = FlyByWire_DWork.PrevY_i[0];
  }

  rateLimiterRate = rtb_BusAssignment_l_pitch_law_protection_attitude_min_eta_dot_pos_s - FlyByWire_DWork.PrevY_i[1];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[1] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[1] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[1] = rtb_BusAssignment_l_pitch_law_protection_attitude_min_eta_dot_pos_s;
  }

  if (FlyByWire_DWork.PrevY_i[1] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[1] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[1] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[1] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[1] = FlyByWire_DWork.PrevY_i[1];
  }

  rateLimiterRate = rtb_BusAssignment_l_pitch_law_protection_attitude_max_eta_dot_pos_s - FlyByWire_DWork.PrevY_i[2];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[2] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[2] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[2] = rtb_BusAssignment_l_pitch_law_protection_attitude_max_eta_dot_pos_s;
  }

  if (FlyByWire_DWork.PrevY_i[2] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_ProportionalGain = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[2] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_ProportionalGain = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_ProportionalGain = FlyByWire_DWork.PrevY_i[2];
  }

  if (FlyByWire_DWork.PrevY_i[2] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[2] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[2] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[2] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[2] = FlyByWire_DWork.PrevY_i[2];
  }

  rtb_inFlight = 0;
  do {
    exitg1 = 0;
    if (rtb_inFlight < 3) {
      if (rtIsNaN(rtb_Saturation[rtb_inFlight])) {
        rtb_ProportionalGain = (rtNaN);
        exitg1 = 1;
      } else {
        rtb_inFlight++;
      }
    } else {
      if (rtIsNaN(rtb_Saturation[1])) {
        condIsTrue = !rtIsNaN(rtb_Saturation[0]);
      } else {
        condIsTrue = ((!rtIsNaN(rtb_Saturation[0])) && (rtb_Saturation[0] < rtb_Saturation[1]));
      }

      if (rtIsNaN(rtb_ProportionalGain)) {
        tmp_2 = !rtIsNaN(rtb_Saturation[1]);
        tmp_0 = !rtIsNaN(rtb_Saturation[0]);
        tmp = tmp_0;
        tmp_1 = tmp_2;
      } else {
        tmp_1 = !rtIsNaN(rtb_Saturation[1]);
        tmp_2 = (tmp_1 && (rtb_Saturation[1] < rtb_ProportionalGain));
        tmp = !rtIsNaN(rtb_Saturation[0]);
        tmp_0 = (tmp && (rtb_Saturation[0] < rtb_ProportionalGain));
        tmp = (tmp && (rtb_Saturation[0] < rtb_ProportionalGain));
        tmp_1 = (tmp_1 && (rtb_Saturation[1] < rtb_ProportionalGain));
      }

      if (condIsTrue) {
        if (tmp_2) {
          rtb_inFlight = 1;
        } else if (tmp_0) {
          rtb_inFlight = 2;
        } else {
          rtb_inFlight = 0;
        }
      } else if (tmp) {
        rtb_inFlight = 0;
      } else if (tmp_1) {
        rtb_inFlight = 2;
      } else {
        rtb_inFlight = 1;
      }

      rtb_ProportionalGain = rtb_Saturation[rtb_inFlight];
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (FlyByWire_DWork.Integration_IC_LOADING != 0) {
    FlyByWire_DWork.Integration_DSTATE = rtb_Gaineta;
    if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
        FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
      }
    }
  }

  if ((rtb_inFlight_e > 0) && (FlyByWire_DWork.Integration_PrevResetState <= 0)) {
    FlyByWire_DWork.Integration_DSTATE = rtb_Gaineta;
    if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
        FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
      }
    }
  }

  if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
    FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
    }
  }

  FlyByWire_Y.out.pitch.law_normal.Cstar_c_g = rtb_Integration13s;
  FlyByWire_Y.out.pitch.law_normal.eta_dot_pos_s = rtb_DiscreteTimeIntegrator;
  rtb_Integration13s = (rtb_BusAssignment_c_pitch_data_computed_in_flight_gain - FlyByWire_P.Constant_Value_i) *
    rtb_Gaineta + FlyByWire_DWork.Integration_DSTATE * rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  if ((rtb_inFlight_e <= 0) && (FlyByWire_DWork.Integration13s_PrevResetState == 1)) {
    FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_IC;
  }

  if (FlyByWire_DWork.Integration13s_DSTATE >= FlyByWire_P.Integration13s_UpperSat) {
    FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration13s_DSTATE <= FlyByWire_P.Integration13s_LowerSat) {
      FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_LowerSat;
    }
  }

  rateLimiterRate = FlyByWire_DWork.Integration13s_DSTATE - FlyByWire_DWork.PrevY_k;
  if (rateLimiterRate > FlyByWire_P.RateLimiter03s_RisingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_k + FlyByWire_P.RateLimiter03s_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiter03s_FallingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_k + FlyByWire_P.RateLimiter03s_FallingLim;
  } else {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.Integration13s_DSTATE;
  }

  FlyByWire_DWork.PrevY_k = rtb_DiscreteTimeIntegrator;
  rtb_BusAssignment_mv_pitch_output_iH_deg = rtb_DiscreteTimeIntegrator;
  if (FlyByWire_DWork.is_active_c5_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c5_FlyByWire = 1U;
    FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
    rtb_inFlight = 0;
  } else if (FlyByWire_DWork.is_c5_FlyByWire == FlyByWire_IN_FlightMode) {
    if (rtb_onGround == 1) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
      rtb_inFlight = 0;
    } else {
      rtb_inFlight = 1;
    }
  } else {
    if (((rtb_onGround == 0) && (rtb_LimiteriH > 8.0)) || (FlyByWire_U.in.data.radio_height_ft > 400.0)) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_FlightMode;
      rtb_inFlight = 1;
    } else {
      rtb_inFlight = 0;
    }
  }

  if (rtb_inFlight > FlyByWire_P.Saturation_UpperSat_p) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Saturation_UpperSat_p;
  } else if (rtb_inFlight < FlyByWire_P.Saturation_LowerSat_h) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Saturation_LowerSat_h;
  } else {
    rtb_DiscreteTimeIntegrator = rtb_inFlight;
  }

  rateLimiterRate = rtb_DiscreteTimeIntegrator - FlyByWire_DWork.PrevY_jv;
  if (rateLimiterRate > FlyByWire_P.RateLimit_RisingLim_b) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_jv + FlyByWire_P.RateLimit_RisingLim_b;
  } else {
    if (rateLimiterRate < FlyByWire_P.RateLimit_FallingLim_m) {
      rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_jv + FlyByWire_P.RateLimit_FallingLim_m;
    }
  }

  FlyByWire_DWork.PrevY_jv = rtb_DiscreteTimeIntegrator;
  rateLimiterRate = FlyByWire_P.Gain1_Gain_m * rtb_Gainxi + look1_binlxpw(rtb_Limitereta,
    FlyByWire_P.BankAngleProtection_bp01Data, FlyByWire_P.BankAngleProtection_tableData, 6U);
  if (rateLimiterRate > FlyByWire_P.RateLimiter_UpperSat) {
    rateLimiterRate = FlyByWire_P.RateLimiter_UpperSat;
  } else {
    if (rateLimiterRate < FlyByWire_P.RateLimiter_LowerSat) {
      rateLimiterRate = FlyByWire_P.RateLimiter_LowerSat;
    }
  }

  rtb_Product = rateLimiterRate * static_cast<real_T>(rtb_inFlight);
  if (FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING != 0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = rtb_Limitereta;
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
        FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if ((rtb_inFlight > 0) && (FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState <= 0)) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = rtb_Limitereta;
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
        FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  rtb_Limitereta = (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE - rtb_Limitereta) * FlyByWire_P.Gain2_Gain +
    FlyByWire_P.Gain1_Gain_mg * rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_pk_deg_s *
    FlyByWire_P.pKp_Gain;
  FlyByWire_Y.out.roll.law_normal.xi_pos = rtb_Limitereta;
  rtb_Limitereta *= rtb_DiscreteTimeIntegrator;
  rtb_BusAssignment_l_roll_output_xi_pos = (rtb_DiscreteTimeIntegrator - FlyByWire_P.Constant_Value_d) * rtb_Gainxi +
    rtb_Limitereta;
  rateLimiterRate = rtb_Integration13s - FlyByWire_DWork.PrevY_d;
  if (rateLimiterRate > FlyByWire_P.RateLimitereta_RisingLim_j) {
    rtb_Limitereta = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_RisingLim_j;
  } else if (rateLimiterRate < FlyByWire_P.RateLimitereta_FallingLim_m) {
    rtb_Limitereta = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_FallingLim_m;
  } else {
    rtb_Limitereta = rtb_Integration13s;
  }

  FlyByWire_DWork.PrevY_d = rtb_Limitereta;
  rtb_Limitereta *= FlyByWire_P.Gaineta_Gain_d;
  if (rtb_Limitereta > FlyByWire_P.Limitereta_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_UpperSat;
  } else if (rtb_Limitereta < FlyByWire_P.Limitereta_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.eta_pos = rtb_Limitereta;
  }

  rateLimiterRate = rtb_BusAssignment_l_roll_output_xi_pos - FlyByWire_DWork.PrevY_jg;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi_RisingLim_n) {
    rtb_Limitereta = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_RisingLim_n;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi_FallingLim_f) {
    rtb_Limitereta = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_FallingLim_f;
  } else {
    rtb_Limitereta = rtb_BusAssignment_l_roll_output_xi_pos;
  }

  FlyByWire_DWork.PrevY_jg = rtb_Limitereta;
  rtb_Limitereta *= FlyByWire_P.Gainxi_Gain_n;
  if (rtb_Limitereta > FlyByWire_P.Limiterxi_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_UpperSat;
  } else if (rtb_Limitereta < FlyByWire_P.Limiterxi_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.xi_pos = rtb_Limitereta;
  }

  FlyByWire_Y.out.sim.raw.data = FlyByWire_U.in.data;
  FlyByWire_Y.out.sim.raw.input = FlyByWire_U.in.input;
  rateLimiterRate = FlyByWire_P.GainiH_Gain * rtb_BusAssignment_mv_pitch_output_iH_deg;
  if (rateLimiterRate > FlyByWire_P.LimiteriH_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.iH_deg = FlyByWire_P.LimiteriH_UpperSat;
  } else if (rateLimiterRate < FlyByWire_P.LimiteriH_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.iH_deg = FlyByWire_P.LimiteriH_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.iH_deg = rateLimiterRate;
  }

  FlyByWire_Y.out.sim.data.nz_g = FlyByWire_U.in.data.nz_g;
  FlyByWire_Y.out.sim.data.Theta_deg = rtb_LimiteriH;
  FlyByWire_Y.out.sim.data.rk_deg_s = FlyByWire_P.Gain_Gain_l * FlyByWire_U.in.data.rk_rad_s * FlyByWire_P.Gainrk_Gain;
  FlyByWire_Y.out.sim.data.pk_deg_s = rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_pk_deg_s;
  FlyByWire_Y.out.sim.data.Vk_kt = FlyByWire_U.in.data.Vk_kt;
  FlyByWire_Y.out.sim.data.radio_height_ft = FlyByWire_U.in.data.radio_height_ft;
  FlyByWire_Y.out.sim.data.CG_percent_MAC = FlyByWire_U.in.data.CG_percent_MAC;
  FlyByWire_Y.out.sim.data_computed.on_ground = rtb_onGround;
  FlyByWire_Y.out.sim.input.delta_eta_pos = rtb_Gaineta;
  FlyByWire_Y.out.sim.input.delta_xi_pos = rtb_Gainxi;
  FlyByWire_Y.out.pitch.data_computed.in_flight = rtb_inFlight_e;
  FlyByWire_Y.out.pitch.data_computed.in_flight_gain = rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  FlyByWire_Y.out.pitch.law_normal.Cstar_g = rtb_Integration;
  FlyByWire_Y.out.pitch.law_protection.attitude_min.eta_dot_pos_s =
    rtb_BusAssignment_l_pitch_law_protection_attitude_min_eta_dot_pos_s;
  FlyByWire_Y.out.pitch.law_protection.attitude_max.eta_dot_pos_s =
    rtb_BusAssignment_l_pitch_law_protection_attitude_max_eta_dot_pos_s;
  FlyByWire_Y.out.pitch.vote.eta_dot_pos_s = rtb_ProportionalGain;
  FlyByWire_Y.out.pitch.integrated.eta_pos = FlyByWire_DWork.Integration_DSTATE;
  FlyByWire_Y.out.pitch.output.eta_pos = rtb_Integration13s;
  FlyByWire_Y.out.pitch.output.iH_deg = rtb_BusAssignment_mv_pitch_output_iH_deg;
  FlyByWire_Y.out.roll.data_computed.in_flight = rtb_inFlight;
  FlyByWire_Y.out.roll.data_computed.in_flight_gain = rtb_DiscreteTimeIntegrator;
  FlyByWire_Y.out.roll.law_normal.pk_c_deg_s = rtb_Product;
  FlyByWire_Y.out.roll.law_normal.Phi_c_deg = FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE;
  FlyByWire_Y.out.roll.output.xi_pos = rtb_BusAssignment_l_roll_output_xi_pos;
  rtb_LimiteriH = FlyByWire_P.Convertto_Gain * FlyByWire_DWork.Integration_DSTATE;
  FlyByWire_DWork.DiscreteTransferFcn1_states = denAccum;
  FlyByWire_DWork.UD_DSTATE = rtb_TSamp;
  FlyByWire_DWork.Integration_IC_LOADING = 0U;
  FlyByWire_DWork.Integration_DSTATE += FlyByWire_P.Integration_gainval * rtb_ProportionalGain;
  if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
    FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
    }
  }

  if (rtb_inFlight_e > 0) {
    FlyByWire_DWork.Integration_PrevResetState = 1;
  } else {
    FlyByWire_DWork.Integration_PrevResetState = 0;
  }

  if ((rtb_inFlight_e == 0) || (FlyByWire_U.in.data.nz_g > 1.25) || (FlyByWire_U.in.data.nz_g < 0.5)) {
    rtb_LimiteriH = FlyByWire_P.Constant_Value;
  }

  FlyByWire_DWork.Integration13s_DSTATE += FlyByWire_P.Integration13s_gainval * rtb_LimiteriH;
  if (FlyByWire_DWork.Integration13s_DSTATE >= FlyByWire_P.Integration13s_UpperSat) {
    FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration13s_DSTATE <= FlyByWire_P.Integration13s_LowerSat) {
      FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_LowerSat;
    }
  }

  if (rtb_inFlight_e > 0) {
    FlyByWire_DWork.Integration13s_PrevResetState = 1;
  } else {
    FlyByWire_DWork.Integration13s_PrevResetState = 0;
  }

  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 0U;
  FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE += FlyByWire_P.DiscreteTimeIntegrator_gainval * rtb_Product;
  if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  if (rtb_inFlight > 0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 1;
  } else {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 0;
  }
}

void FlyByWireModelClass::initialize()
{
  rt_InitInfAndNaN(sizeof(real_T));
  (void) std::memset(static_cast<void *>(&FlyByWire_DWork), 0,
                     sizeof(D_Work_FlyByWire_T));
  FlyByWire_U.in = FlyByWire_rtZfbw_input;
  FlyByWire_Y.out = FlyByWire_rtZfbw_output;
  FlyByWire_DWork.PrevY = FlyByWire_P.RateLimitereta_IC;
  FlyByWire_DWork.PrevY_c = FlyByWire_P.RateLimiterxi_IC;
  FlyByWire_DWork.PrevY_h = FlyByWire_P.RateLimit_IC;
  FlyByWire_DWork.DiscreteTransferFcn1_states = FlyByWire_P.DiscreteTransferFcn1_InitialStates;
  FlyByWire_DWork.PrevY_j = FlyByWire_P.RateLimiter_IC;
  FlyByWire_DWork.UD_DSTATE = FlyByWire_P.PID_DifferentiatorICPrevScaledInput;
  FlyByWire_DWork.PrevY_i[0] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.PrevY_i[1] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.PrevY_i[2] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.Integration_IC_LOADING = 1U;
  FlyByWire_DWork.Integration_PrevResetState = 2;
  FlyByWire_DWork.Integration13s_DSTATE = FlyByWire_P.Integration13s_IC;
  FlyByWire_DWork.Integration13s_PrevResetState = 2;
  FlyByWire_DWork.PrevY_k = FlyByWire_P.RateLimiter03s_IC;
  FlyByWire_DWork.PrevY_jv = FlyByWire_P.RateLimit_IC_d;
  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 1U;
  FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 2;
  FlyByWire_DWork.PrevY_d = FlyByWire_P.RateLimitereta_IC_c;
  FlyByWire_DWork.PrevY_jg = FlyByWire_P.RateLimiterxi_IC_c;
  FlyByWire_DWork.is_active_c1_FlyByWire = 0U;
  FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c3_FlyByWire = 0U;
  FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
  FlyByWire_DWork.is_active_c5_FlyByWire = 0U;
  FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
}

void FlyByWireModelClass::terminate()
{
}

FlyByWireModelClass::FlyByWireModelClass()
{
}

FlyByWireModelClass::~FlyByWireModelClass()
{
}
