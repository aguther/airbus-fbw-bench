#include "fbw.h"
#include "fbw_private.h"

const uint8_T fbw_IN_InAir = 1U;
const uint8_T fbw_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T fbw_IN_OnGround = 2U;
const uint8_T fbw_IN_Flare = 1U;
const uint8_T fbw_IN_Flight_High = 2U;
const uint8_T fbw_IN_Flight_Low = 3U;
const uint8_T fbw_IN_Ground = 4U;
const uint8_T fbw_IN_FlightMode = 1U;
const uint8_T fbw_IN_GroundMode = 2U;
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

void fbwModelClass::step()
{
  real_T rateLimiterRate;
  real_T denAccum;
  boolean_T condIsTrue;
  int32_T rtb_onGround;
  int32_T rtb_inFlight_h;
  int32_T rtb_inFlight;
  real_T rtb_TSamp;
  real_T rtb_Saturation_g[3];
  boolean_T tmp;
  boolean_T tmp_0;
  boolean_T tmp_1;
  boolean_T tmp_2;
  int32_T exitg1;
  fbw_Y.out_sim_simdata_qk_deg_s = fbw_P.Gain_Gain * fbw_U.in_sim_simrawdata_qk_rad_s * fbw_P.Gainqk_Gain;
  fbw_Y.out_pitch_pitchnormal_Cstar_g = fbw_P.Gain1_Gain * fbw_Y.out_sim_simdata_qk_deg_s * fbw_P.Constant2_Value +
    fbw_U.in_sim_simrawdata_nz_g;
  fbw_Y.out_sim_simdata_Theta_deg = fbw_P.GainTheta_Gain * fbw_U.in_sim_simrawdata_Theta_deg;
  fbw_Y.out_sim_simdata_Phi_deg = fbw_P.GainPhi_Gain * fbw_U.in_sim_simrawdata_Phi_deg;
  rateLimiterRate = fbw_U.in_sim_simrawinput_delta_eta_pos - fbw_DWork.PrevY;
  if (rateLimiterRate > fbw_P.RateLimitereta_RisingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY + fbw_P.RateLimitereta_RisingLim;
  } else if (rateLimiterRate < fbw_P.RateLimitereta_FallingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY + fbw_P.RateLimitereta_FallingLim;
  } else {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_U.in_sim_simrawinput_delta_eta_pos;
  }

  fbw_DWork.PrevY = fbw_Y.out_sim_simrawoutput_xi_pos;
  fbw_Y.out_sim_siminput_delta_eta_pos = fbw_P.Gaineta_Gain * fbw_Y.out_sim_simrawoutput_xi_pos;
  rateLimiterRate = fbw_Y.out_sim_siminput_delta_eta_pos - fbw_DWork.PrevY_e;
  if (rateLimiterRate > fbw_P.RateLimiter_RisingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_e + fbw_P.RateLimiter_RisingLim;
  } else if (rateLimiterRate < fbw_P.RateLimiter_FallingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_e + fbw_P.RateLimiter_FallingLim;
  } else {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_Y.out_sim_siminput_delta_eta_pos;
  }

  fbw_DWork.PrevY_e = fbw_Y.out_sim_simrawoutput_xi_pos;
  fbw_Y.out_sim_simrawoutput_xi_pos = look1_binlxpw(fbw_Y.out_sim_simrawoutput_xi_pos, fbw_P.loaddemand_bp01Data,
    fbw_P.loaddemand_tableData, 2U);
  fbw_Y.out_sim_simrawoutput_xi_pos += std::cos(fbw_P.Gain1_Gain_c * fbw_Y.out_sim_simdata_Theta_deg) / std::cos
    (fbw_P.Gain1_Gain_ci * fbw_Y.out_sim_simdata_Phi_deg);
  if (fbw_Y.out_sim_simrawoutput_xi_pos > fbw_P.Saturation_UpperSat) {
    fbw_Y.out_pitch_pitchnormal_Cstar_c_g = fbw_P.Saturation_UpperSat;
  } else if (fbw_Y.out_sim_simrawoutput_xi_pos < fbw_P.Saturation_LowerSat) {
    fbw_Y.out_pitch_pitchnormal_Cstar_c_g = fbw_P.Saturation_LowerSat;
  } else {
    fbw_Y.out_pitch_pitchnormal_Cstar_c_g = fbw_Y.out_sim_simrawoutput_xi_pos;
  }

  fbw_Y.out_sim_simrawoutput_xi_pos = fbw_Y.out_pitch_pitchnormal_Cstar_g - fbw_Y.out_pitch_pitchnormal_Cstar_c_g;
  rtb_TSamp = fbw_P.PID_D * fbw_Y.out_sim_simrawoutput_xi_pos * fbw_P.TSamp_WtEt;
  fbw_Y.out_sim_simrawoutput_eta_pos = fbw_P.PID_P * fbw_Y.out_sim_simrawoutput_xi_pos + (rtb_TSamp -
    fbw_DWork.UD_DSTATE);
  if (fbw_Y.out_sim_simrawoutput_eta_pos > fbw_P.PID_UpperSaturationLimit) {
    fbw_Y.out_pitch_pitchnormal_eta_dot_pos_s = fbw_P.PID_UpperSaturationLimit;
  } else if (fbw_Y.out_sim_simrawoutput_eta_pos < fbw_P.PID_LowerSaturationLimit) {
    fbw_Y.out_pitch_pitchnormal_eta_dot_pos_s = fbw_P.PID_LowerSaturationLimit;
  } else {
    fbw_Y.out_pitch_pitchnormal_eta_dot_pos_s = fbw_Y.out_sim_simrawoutput_eta_pos;
  }

  denAccum = (fbw_Y.out_sim_simdata_qk_deg_s - fbw_P.DiscreteTransferFcn1_DenCoef[1] *
              fbw_DWork.DiscreteTransferFcn1_states) / fbw_P.DiscreteTransferFcn1_DenCoef[0];
  fbw_Y.out_sim_simrawoutput_eta_pos = (fbw_P.DiscreteTransferFcn1_NumCoef[0] * denAccum +
    fbw_P.DiscreteTransferFcn1_NumCoef[1] * fbw_DWork.DiscreteTransferFcn1_states) * fbw_P.K7857_Gain + fbw_P.K7958_Gain
    * fbw_Y.out_sim_simdata_qk_deg_s;
  fbw_Y.out_sim_simrawoutput_xi_pos = look1_binlxpw(fbw_U.in_sim_simrawdata_Vk_kt, fbw_P.F193LUT_bp01Data,
    fbw_P.F193LUT_tableData, 7U);
  fbw_Y.out_pitch_pitchprotectionattitudemin_eta_dot_pos_s = ((fbw_P.u1_Value + fbw_Y.out_sim_simdata_Theta_deg) *
    fbw_P.K7659_Gain + fbw_Y.out_sim_simrawoutput_eta_pos) * fbw_Y.out_sim_simrawoutput_xi_pos;
  fbw_Y.out_pitch_pitchprotectionattitudemax_eta_dot_pos_s = ((fbw_Y.out_sim_simdata_Theta_deg - fbw_P.u7_Value) *
    fbw_P.K7765_Gain + fbw_Y.out_sim_simrawoutput_eta_pos) * fbw_Y.out_sim_simrawoutput_xi_pos;
  rateLimiterRate = fbw_Y.out_pitch_pitchnormal_eta_dot_pos_s - fbw_DWork.PrevY_g[0];
  if (rateLimiterRate > fbw_P.Limiter_RisingLim) {
    fbw_DWork.PrevY_g[0] += fbw_P.Limiter_RisingLim;
  } else if (rateLimiterRate < fbw_P.Limiter_FallingLim) {
    fbw_DWork.PrevY_g[0] += fbw_P.Limiter_FallingLim;
  } else {
    fbw_DWork.PrevY_g[0] = fbw_Y.out_pitch_pitchnormal_eta_dot_pos_s;
  }

  if (fbw_DWork.PrevY_g[0] > fbw_P.Saturation_UpperSat_k) {
    rtb_Saturation_g[0] = fbw_P.Saturation_UpperSat_k;
  } else if (fbw_DWork.PrevY_g[0] < fbw_P.Saturation_LowerSat_d) {
    rtb_Saturation_g[0] = fbw_P.Saturation_LowerSat_d;
  } else {
    rtb_Saturation_g[0] = fbw_DWork.PrevY_g[0];
  }

  rateLimiterRate = fbw_Y.out_pitch_pitchprotectionattitudemin_eta_dot_pos_s - fbw_DWork.PrevY_g[1];
  if (rateLimiterRate > fbw_P.Limiter_RisingLim) {
    fbw_DWork.PrevY_g[1] += fbw_P.Limiter_RisingLim;
  } else if (rateLimiterRate < fbw_P.Limiter_FallingLim) {
    fbw_DWork.PrevY_g[1] += fbw_P.Limiter_FallingLim;
  } else {
    fbw_DWork.PrevY_g[1] = fbw_Y.out_pitch_pitchprotectionattitudemin_eta_dot_pos_s;
  }

  if (fbw_DWork.PrevY_g[1] > fbw_P.Saturation_UpperSat_k) {
    rtb_Saturation_g[1] = fbw_P.Saturation_UpperSat_k;
  } else if (fbw_DWork.PrevY_g[1] < fbw_P.Saturation_LowerSat_d) {
    rtb_Saturation_g[1] = fbw_P.Saturation_LowerSat_d;
  } else {
    rtb_Saturation_g[1] = fbw_DWork.PrevY_g[1];
  }

  rateLimiterRate = fbw_Y.out_pitch_pitchprotectionattitudemax_eta_dot_pos_s - fbw_DWork.PrevY_g[2];
  if (rateLimiterRate > fbw_P.Limiter_RisingLim) {
    fbw_DWork.PrevY_g[2] += fbw_P.Limiter_RisingLim;
  } else if (rateLimiterRate < fbw_P.Limiter_FallingLim) {
    fbw_DWork.PrevY_g[2] += fbw_P.Limiter_FallingLim;
  } else {
    fbw_DWork.PrevY_g[2] = fbw_Y.out_pitch_pitchprotectionattitudemax_eta_dot_pos_s;
  }

  if (fbw_DWork.PrevY_g[2] > fbw_P.Saturation_UpperSat_k) {
    rateLimiterRate = fbw_P.Saturation_UpperSat_k;
  } else if (fbw_DWork.PrevY_g[2] < fbw_P.Saturation_LowerSat_d) {
    rateLimiterRate = fbw_P.Saturation_LowerSat_d;
  } else {
    rateLimiterRate = fbw_DWork.PrevY_g[2];
  }

  if (fbw_DWork.PrevY_g[2] > fbw_P.Saturation_UpperSat_k) {
    rtb_Saturation_g[2] = fbw_P.Saturation_UpperSat_k;
  } else if (fbw_DWork.PrevY_g[2] < fbw_P.Saturation_LowerSat_d) {
    rtb_Saturation_g[2] = fbw_P.Saturation_LowerSat_d;
  } else {
    rtb_Saturation_g[2] = fbw_DWork.PrevY_g[2];
  }

  rtb_onGround = 0;
  do {
    exitg1 = 0;
    if (rtb_onGround < 3) {
      if (rtIsNaN(rtb_Saturation_g[rtb_onGround])) {
        fbw_Y.out_pitch_pitchvote_eta_dot_pos_s = (rtNaN);
        exitg1 = 1;
      } else {
        rtb_onGround++;
      }
    } else {
      if (rtIsNaN(rtb_Saturation_g[1])) {
        condIsTrue = !rtIsNaN(rtb_Saturation_g[0]);
      } else {
        condIsTrue = ((!rtIsNaN(rtb_Saturation_g[0])) && (rtb_Saturation_g[0] < rtb_Saturation_g[1]));
      }

      if (rtIsNaN(rateLimiterRate)) {
        tmp_2 = !rtIsNaN(rtb_Saturation_g[1]);
        tmp_0 = !rtIsNaN(rtb_Saturation_g[0]);
        tmp = tmp_0;
        tmp_1 = tmp_2;
      } else {
        tmp_1 = !rtIsNaN(rtb_Saturation_g[1]);
        tmp_2 = (tmp_1 && (rtb_Saturation_g[1] < rateLimiterRate));
        tmp = !rtIsNaN(rtb_Saturation_g[0]);
        tmp_0 = (tmp && (rtb_Saturation_g[0] < rateLimiterRate));
        tmp = (tmp && (rtb_Saturation_g[0] < rateLimiterRate));
        tmp_1 = (tmp_1 && (rtb_Saturation_g[1] < rateLimiterRate));
      }

      if (condIsTrue) {
        if (tmp_2) {
          rtb_onGround = 1;
        } else if (tmp_0) {
          rtb_onGround = 2;
        } else {
          rtb_onGround = 0;
        }
      } else if (tmp) {
        rtb_onGround = 0;
      } else if (tmp_1) {
        rtb_onGround = 2;
      } else {
        rtb_onGround = 1;
      }

      fbw_Y.out_pitch_pitchvote_eta_dot_pos_s = rtb_Saturation_g[rtb_onGround];
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (fbw_DWork.is_active_c1_fbw == 0U) {
    fbw_DWork.is_active_c1_fbw = 1U;
    fbw_DWork.is_c1_fbw = fbw_IN_OnGround;
    rtb_onGround = 1;
  } else if (fbw_DWork.is_c1_fbw == fbw_IN_InAir) {
    if (fbw_U.in_sim_simrawdata_radio_height_ft <= 10.0) {
      fbw_DWork.is_c1_fbw = fbw_IN_OnGround;
      rtb_onGround = 1;
    } else {
      rtb_onGround = 0;
    }
  } else {
    if (fbw_U.in_sim_simrawdata_radio_height_ft > 10.0) {
      fbw_DWork.is_c1_fbw = fbw_IN_InAir;
      rtb_onGround = 0;
    } else {
      rtb_onGround = 1;
    }
  }

  fbw_DWork.chartAbsoluteTimeCounter++;
  condIsTrue = ((rtb_onGround == 1) && (fbw_Y.out_sim_simdata_Theta_deg < 2.5));
  if ((!condIsTrue) || (!fbw_DWork.condWasTrueAtLastTimeStep_1)) {
    fbw_DWork.durationLastReferenceTick_1 = fbw_DWork.chartAbsoluteTimeCounter;
  }

  fbw_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
  if (fbw_DWork.is_active_c3_fbw == 0U) {
    fbw_DWork.chartAbsoluteTimeCounter = 0;
    fbw_DWork.is_active_c3_fbw = 1U;
    fbw_DWork.is_c3_fbw = fbw_IN_Ground;
    rtb_inFlight_h = 0;
    fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
  } else {
    switch (fbw_DWork.is_c3_fbw) {
     case fbw_IN_Flare:
      condIsTrue = ((rtb_onGround == 1) && (fbw_Y.out_sim_simdata_Theta_deg < 2.5));
      if ((!condIsTrue) || (!fbw_DWork.condWasTrueAtLastTimeStep_1)) {
        fbw_DWork.durationLastReferenceTick_1 = fbw_DWork.chartAbsoluteTimeCounter;
      }

      fbw_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
      if (fbw_DWork.chartAbsoluteTimeCounter - fbw_DWork.durationLastReferenceTick_1 > 250) {
        fbw_DWork.is_c3_fbw = fbw_IN_Ground;
        rtb_inFlight_h = 0;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      } else if ((fbw_U.in_sim_simrawdata_radio_height_ft > 50.0) && (fbw_Y.out_sim_simdata_Theta_deg > 8.0)) {
        fbw_DWork.is_c3_fbw = fbw_IN_Flight_Low;
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      } else {
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 1.0;
      }
      break;

     case fbw_IN_Flight_High:
      if (fbw_U.in_sim_simrawdata_radio_height_ft <= 50.0) {
        fbw_DWork.durationLastReferenceTick_1 = fbw_DWork.chartAbsoluteTimeCounter;
        fbw_DWork.is_c3_fbw = fbw_IN_Flare;
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 1.0;
        fbw_DWork.condWasTrueAtLastTimeStep_1 = ((rtb_onGround == 1) && (fbw_Y.out_sim_simdata_Theta_deg < 2.5));
      } else {
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      }
      break;

     case fbw_IN_Flight_Low:
      if (fbw_U.in_sim_simrawdata_radio_height_ft > 50.0) {
        fbw_DWork.is_c3_fbw = fbw_IN_Flight_High;
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      } else {
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      }
      break;

     default:
      if (((rtb_onGround == 0) && (fbw_Y.out_sim_simdata_Theta_deg > 8.0)) || (fbw_U.in_sim_simrawdata_radio_height_ft >
           400.0)) {
        fbw_DWork.is_c3_fbw = fbw_IN_Flight_Low;
        rtb_inFlight_h = 1;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      } else {
        rtb_inFlight_h = 0;
        fbw_Y.out_pitch_pitchdatacomputed_in_flare = 0.0;
      }
      break;
    }
  }

  if (fbw_DWork.Integration_IC_LOADING != 0) {
    fbw_DWork.Integration_DSTATE = fbw_Y.out_sim_siminput_delta_eta_pos;
    if (fbw_DWork.Integration_DSTATE >= fbw_P.Integration_UpperSat) {
      fbw_DWork.Integration_DSTATE = fbw_P.Integration_UpperSat;
    } else {
      if (fbw_DWork.Integration_DSTATE <= fbw_P.Integration_LowerSat) {
        fbw_DWork.Integration_DSTATE = fbw_P.Integration_LowerSat;
      }
    }
  }

  if ((rtb_inFlight_h > 0) && (fbw_DWork.Integration_PrevResetState <= 0)) {
    fbw_DWork.Integration_DSTATE = fbw_Y.out_sim_siminput_delta_eta_pos;
    if (fbw_DWork.Integration_DSTATE >= fbw_P.Integration_UpperSat) {
      fbw_DWork.Integration_DSTATE = fbw_P.Integration_UpperSat;
    } else {
      if (fbw_DWork.Integration_DSTATE <= fbw_P.Integration_LowerSat) {
        fbw_DWork.Integration_DSTATE = fbw_P.Integration_LowerSat;
      }
    }
  }

  if (fbw_DWork.Integration_DSTATE >= fbw_P.Integration_UpperSat) {
    fbw_DWork.Integration_DSTATE = fbw_P.Integration_UpperSat;
  } else {
    if (fbw_DWork.Integration_DSTATE <= fbw_P.Integration_LowerSat) {
      fbw_DWork.Integration_DSTATE = fbw_P.Integration_LowerSat;
    }
  }

  fbw_Y.out_pitch_pitchintegrated_eta_pos = fbw_DWork.Integration_DSTATE;
  if (rtb_inFlight_h > fbw_P.Saturation_UpperSat_ks) {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_P.Saturation_UpperSat_ks;
  } else if (rtb_inFlight_h < fbw_P.Saturation_LowerSat_k) {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_P.Saturation_LowerSat_k;
  } else {
    fbw_Y.out_sim_simrawoutput_eta_pos = rtb_inFlight_h;
  }

  rateLimiterRate = fbw_Y.out_sim_simrawoutput_eta_pos - fbw_DWork.PrevY_p;
  if (rateLimiterRate > fbw_P.RateLimit_RisingLim) {
    fbw_DWork.PrevY_p += fbw_P.RateLimit_RisingLim;
  } else if (rateLimiterRate < fbw_P.RateLimit_FallingLim) {
    fbw_DWork.PrevY_p += fbw_P.RateLimit_FallingLim;
  } else {
    fbw_DWork.PrevY_p = fbw_Y.out_sim_simrawoutput_eta_pos;
  }

  fbw_Y.out_pitch_pitchoutput_eta_pos = (fbw_DWork.PrevY_p - fbw_P.Constant_Value_n) *
    fbw_Y.out_sim_siminput_delta_eta_pos + fbw_DWork.Integration_DSTATE * fbw_DWork.PrevY_p;
  rateLimiterRate = fbw_Y.out_pitch_pitchoutput_eta_pos - fbw_DWork.PrevY_d;
  if (rateLimiterRate > fbw_P.RateLimitereta_RisingLim_j) {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_DWork.PrevY_d + fbw_P.RateLimitereta_RisingLim_j;
  } else if (rateLimiterRate < fbw_P.RateLimitereta_FallingLim_m) {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_DWork.PrevY_d + fbw_P.RateLimitereta_FallingLim_m;
  } else {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_Y.out_pitch_pitchoutput_eta_pos;
  }

  fbw_DWork.PrevY_d = fbw_Y.out_sim_simrawoutput_eta_pos;
  fbw_Y.out_sim_simrawoutput_eta_pos *= fbw_P.Gaineta_Gain_d;
  if ((rtb_inFlight_h <= 0) && (fbw_DWork.Integration13s_PrevResetState == 1)) {
    fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_IC;
  }

  if (fbw_DWork.Integration13s_DSTATE >= fbw_P.Integration13s_UpperSat) {
    fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_UpperSat;
  } else {
    if (fbw_DWork.Integration13s_DSTATE <= fbw_P.Integration13s_LowerSat) {
      fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_LowerSat;
    }
  }

  rateLimiterRate = fbw_DWork.Integration13s_DSTATE - fbw_DWork.PrevY_n;
  if (rateLimiterRate > fbw_P.RateLimiter03s_RisingLim) {
    fbw_DWork.PrevY_n += fbw_P.RateLimiter03s_RisingLim;
  } else if (rateLimiterRate < fbw_P.RateLimiter03s_FallingLim) {
    fbw_DWork.PrevY_n += fbw_P.RateLimiter03s_FallingLim;
  } else {
    fbw_DWork.PrevY_n = fbw_DWork.Integration13s_DSTATE;
  }

  if (fbw_DWork.is_active_c5_fbw == 0U) {
    fbw_DWork.is_active_c5_fbw = 1U;
    fbw_DWork.is_c5_fbw = fbw_IN_GroundMode;
    rtb_inFlight = 0;
  } else if (fbw_DWork.is_c5_fbw == fbw_IN_FlightMode) {
    if (rtb_onGround == 1) {
      fbw_DWork.is_c5_fbw = fbw_IN_GroundMode;
      rtb_inFlight = 0;
    } else {
      rtb_inFlight = 1;
    }
  } else {
    if (((rtb_onGround == 0) && (fbw_Y.out_sim_simdata_Theta_deg > 8.0)) || (fbw_U.in_sim_simrawdata_radio_height_ft >
         400.0)) {
      fbw_DWork.is_c5_fbw = fbw_IN_FlightMode;
      rtb_inFlight = 1;
    } else {
      rtb_inFlight = 0;
    }
  }

  if (rtb_inFlight > fbw_P.Saturation_UpperSat_f) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_P.Saturation_UpperSat_f;
  } else if (rtb_inFlight < fbw_P.Saturation_LowerSat_n) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_P.Saturation_LowerSat_n;
  } else {
    fbw_Y.out_sim_simrawoutput_xi_pos = rtb_inFlight;
  }

  rateLimiterRate = fbw_Y.out_sim_simrawoutput_xi_pos - fbw_DWork.PrevY_gj;
  if (rateLimiterRate > fbw_P.RateLimit_RisingLim_e) {
    fbw_DWork.PrevY_gj += fbw_P.RateLimit_RisingLim_e;
  } else if (rateLimiterRate < fbw_P.RateLimit_FallingLim_i) {
    fbw_DWork.PrevY_gj += fbw_P.RateLimit_FallingLim_i;
  } else {
    fbw_DWork.PrevY_gj = fbw_Y.out_sim_simrawoutput_xi_pos;
  }

  rateLimiterRate = fbw_U.in_sim_simrawinput_delta_xi_pos - fbw_DWork.PrevY_c;
  if (rateLimiterRate > fbw_P.RateLimiterxi_RisingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_c + fbw_P.RateLimiterxi_RisingLim;
  } else if (rateLimiterRate < fbw_P.RateLimiterxi_FallingLim) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_c + fbw_P.RateLimiterxi_FallingLim;
  } else {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_U.in_sim_simrawinput_delta_xi_pos;
  }

  fbw_DWork.PrevY_c = fbw_Y.out_sim_simrawoutput_xi_pos;
  fbw_Y.out_sim_siminput_delta_xi_pos = fbw_P.Gainxi_Gain * fbw_Y.out_sim_simrawoutput_xi_pos;
  if (fbw_DWork.DiscreteTimeIntegrator_IC_LOADING != 0) {
    fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_Y.out_sim_simdata_Phi_deg;
    if (fbw_DWork.DiscreteTimeIntegrator_DSTATE >= fbw_P.DiscreteTimeIntegrator_UpperSat) {
      fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (fbw_DWork.DiscreteTimeIntegrator_DSTATE <= fbw_P.DiscreteTimeIntegrator_LowerSat) {
        fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if ((rtb_inFlight > 0) && (fbw_DWork.DiscreteTimeIntegrator_PrevResetState <= 0)) {
    fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_Y.out_sim_simdata_Phi_deg;
    if (fbw_DWork.DiscreteTimeIntegrator_DSTATE >= fbw_P.DiscreteTimeIntegrator_UpperSat) {
      fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (fbw_DWork.DiscreteTimeIntegrator_DSTATE <= fbw_P.DiscreteTimeIntegrator_LowerSat) {
        fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if (fbw_DWork.DiscreteTimeIntegrator_DSTATE >= fbw_P.DiscreteTimeIntegrator_UpperSat) {
    fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (fbw_DWork.DiscreteTimeIntegrator_DSTATE <= fbw_P.DiscreteTimeIntegrator_LowerSat) {
      fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  fbw_Y.out_roll_rollnormal_Phi_c_deg = fbw_DWork.DiscreteTimeIntegrator_DSTATE;
  fbw_Y.out_sim_simdata_pk_deg_s = fbw_P.Gain_Gain_a * fbw_U.in_sim_simrawdata_pk_rad_s * fbw_P.Gainpk_Gain;
  fbw_Y.out_roll_rollnormal_xi_pos = (fbw_DWork.DiscreteTimeIntegrator_DSTATE - fbw_Y.out_sim_simdata_Phi_deg) *
    fbw_P.Gain2_Gain + fbw_P.Gain1_Gain_g * fbw_Y.out_sim_simdata_pk_deg_s * fbw_P.pKp_Gain;
  fbw_Y.out_roll_rolloutput_xi_pos = (fbw_DWork.PrevY_gj - fbw_P.Constant_Value_b) * fbw_Y.out_sim_siminput_delta_xi_pos
    + fbw_Y.out_roll_rollnormal_xi_pos * fbw_DWork.PrevY_gj;
  rateLimiterRate = fbw_Y.out_roll_rolloutput_xi_pos - fbw_DWork.PrevY_j;
  if (rateLimiterRate > fbw_P.RateLimiterxi_RisingLim_n) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_j + fbw_P.RateLimiterxi_RisingLim_n;
  } else if (rateLimiterRate < fbw_P.RateLimiterxi_FallingLim_f) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_DWork.PrevY_j + fbw_P.RateLimiterxi_FallingLim_f;
  } else {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_Y.out_roll_rolloutput_xi_pos;
  }

  fbw_DWork.PrevY_j = fbw_Y.out_sim_simrawoutput_xi_pos;
  fbw_Y.out_sim_simrawoutput_xi_pos *= fbw_P.Gainxi_Gain_n;
  rateLimiterRate = fbw_P.Gain1_Gain_h * fbw_Y.out_sim_siminput_delta_xi_pos + look1_binlxpw
    (fbw_Y.out_sim_simdata_Phi_deg, fbw_P.BankAngleProtection_bp01Data, fbw_P.BankAngleProtection_tableData, 6U);
  if (rateLimiterRate > fbw_P.RateLimiter_UpperSat) {
    rateLimiterRate = fbw_P.RateLimiter_UpperSat;
  } else {
    if (rateLimiterRate < fbw_P.RateLimiter_LowerSat) {
      rateLimiterRate = fbw_P.RateLimiter_LowerSat;
    }
  }

  fbw_Y.out_roll_rollnormal_pk_c_deg_s = rateLimiterRate * static_cast<real_T>(rtb_inFlight);
  fbw_DWork.UD_DSTATE = rtb_TSamp;
  fbw_DWork.DiscreteTransferFcn1_states = denAccum;
  fbw_DWork.Integration_IC_LOADING = 0U;
  fbw_DWork.Integration_DSTATE += fbw_P.Integration_gainval * fbw_Y.out_pitch_pitchvote_eta_dot_pos_s;
  if (fbw_DWork.Integration_DSTATE >= fbw_P.Integration_UpperSat) {
    fbw_DWork.Integration_DSTATE = fbw_P.Integration_UpperSat;
  } else {
    if (fbw_DWork.Integration_DSTATE <= fbw_P.Integration_LowerSat) {
      fbw_DWork.Integration_DSTATE = fbw_P.Integration_LowerSat;
    }
  }

  if (rtb_inFlight_h > 0) {
    fbw_DWork.Integration_PrevResetState = 1;
  } else {
    fbw_DWork.Integration_PrevResetState = 0;
  }

  if ((rtb_inFlight_h == 0) || (fbw_U.in_sim_simrawdata_nz_g > 1.25) || (fbw_U.in_sim_simrawdata_nz_g < 0.5)) {
    rtb_TSamp = fbw_P.Constant_Value;
  } else {
    rtb_TSamp = fbw_P.Convertto_Gain * fbw_Y.out_pitch_pitchoutput_eta_pos;
  }

  fbw_DWork.Integration13s_DSTATE += fbw_P.Integration13s_gainval * rtb_TSamp;
  if (fbw_DWork.Integration13s_DSTATE >= fbw_P.Integration13s_UpperSat) {
    fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_UpperSat;
  } else {
    if (fbw_DWork.Integration13s_DSTATE <= fbw_P.Integration13s_LowerSat) {
      fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_LowerSat;
    }
  }

  if (rtb_inFlight_h > 0) {
    fbw_DWork.Integration13s_PrevResetState = 1;
  } else {
    fbw_DWork.Integration13s_PrevResetState = 0;
  }

  fbw_DWork.DiscreteTimeIntegrator_IC_LOADING = 0U;
  fbw_DWork.DiscreteTimeIntegrator_DSTATE += fbw_P.DiscreteTimeIntegrator_gainval * fbw_Y.out_roll_rollnormal_pk_c_deg_s;
  if (fbw_DWork.DiscreteTimeIntegrator_DSTATE >= fbw_P.DiscreteTimeIntegrator_UpperSat) {
    fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (fbw_DWork.DiscreteTimeIntegrator_DSTATE <= fbw_P.DiscreteTimeIntegrator_LowerSat) {
      fbw_DWork.DiscreteTimeIntegrator_DSTATE = fbw_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  if (rtb_inFlight > 0) {
    fbw_DWork.DiscreteTimeIntegrator_PrevResetState = 1;
  } else {
    fbw_DWork.DiscreteTimeIntegrator_PrevResetState = 0;
  }

  fbw_Y.out_sim_simrawdata_nz_g = fbw_U.in_sim_simrawdata_nz_g;
  fbw_Y.out_sim_simrawdata_Theta_deg = fbw_U.in_sim_simrawdata_Theta_deg;
  fbw_Y.out_sim_simrawdata_Phi_deg = fbw_U.in_sim_simrawdata_Phi_deg;
  fbw_Y.out_sim_simrawdata_qk_rad_s = fbw_U.in_sim_simrawdata_qk_rad_s;
  fbw_Y.out_sim_simrawdata_rk_rad_s = fbw_U.in_sim_simrawdata_rk_rad_s;
  fbw_Y.out_sim_simrawdata_pk_rad_s = fbw_U.in_sim_simrawdata_pk_rad_s;
  fbw_Y.out_sim_simrawdata_Vk_kt = fbw_U.in_sim_simrawdata_Vk_kt;
  fbw_Y.out_sim_simrawdata_radio_height_ft = fbw_U.in_sim_simrawdata_radio_height_ft;
  fbw_Y.out_sim_simrawdata_CG_percent_MAC = fbw_U.in_sim_simrawdata_CG_percent_MAC;
  fbw_Y.out_sim_simrawinput_delta_eta_pos = fbw_U.in_sim_simrawinput_delta_eta_pos;
  fbw_Y.out_sim_simrawinput_delta_xi_pos = fbw_U.in_sim_simrawinput_delta_xi_pos;
  fbw_Y.out_sim_simdata_nz_g = fbw_U.in_sim_simrawdata_nz_g;
  fbw_Y.out_sim_simdata_rk_deg_s = fbw_P.Gain_Gain_l * fbw_U.in_sim_simrawdata_rk_rad_s * fbw_P.Gainrk_Gain;
  fbw_Y.out_sim_simdata_Vk_kt = fbw_U.in_sim_simrawdata_Vk_kt;
  fbw_Y.out_sim_simdata_radio_height_ft = fbw_U.in_sim_simrawdata_radio_height_ft;
  fbw_Y.out_sim_simdata_CG_percent_MAC = fbw_U.in_sim_simrawdata_CG_percent_MAC;
  fbw_Y.out_sim_simdatacomputed_on_ground = rtb_onGround;
  if (fbw_Y.out_sim_simrawoutput_eta_pos > fbw_P.Limitereta_UpperSat) {
    fbw_Y.out_sim_simrawoutput_eta_pos = fbw_P.Limitereta_UpperSat;
  } else {
    if (fbw_Y.out_sim_simrawoutput_eta_pos < fbw_P.Limitereta_LowerSat) {
      fbw_Y.out_sim_simrawoutput_eta_pos = fbw_P.Limitereta_LowerSat;
    }
  }

  fbw_Y.out_sim_simrawoutput_iH_deg = fbw_P.GainiH_Gain * fbw_DWork.PrevY_n;
  if (fbw_Y.out_sim_simrawoutput_iH_deg > fbw_P.LimiteriH_UpperSat) {
    fbw_Y.out_sim_simrawoutput_iH_deg = fbw_P.LimiteriH_UpperSat;
  } else {
    if (fbw_Y.out_sim_simrawoutput_iH_deg < fbw_P.LimiteriH_LowerSat) {
      fbw_Y.out_sim_simrawoutput_iH_deg = fbw_P.LimiteriH_LowerSat;
    }
  }

  if (fbw_Y.out_sim_simrawoutput_xi_pos > fbw_P.Limiterxi_UpperSat) {
    fbw_Y.out_sim_simrawoutput_xi_pos = fbw_P.Limiterxi_UpperSat;
  } else {
    if (fbw_Y.out_sim_simrawoutput_xi_pos < fbw_P.Limiterxi_LowerSat) {
      fbw_Y.out_sim_simrawoutput_xi_pos = fbw_P.Limiterxi_LowerSat;
    }
  }

  fbw_Y.out_pitch_pitchdatacomputed_in_flight = rtb_inFlight_h;
  fbw_Y.out_pitch_pitchdatacomputed_in_flight_gain = fbw_DWork.PrevY_p;
  fbw_Y.out_pitch_pitchoutput_iH_deg = fbw_DWork.PrevY_n;
  fbw_Y.out_roll_rolldatacomputed_in_flight = rtb_inFlight;
  fbw_Y.out_roll_rolldatacomputed_in_flight_gain = fbw_DWork.PrevY_gj;
}

void fbwModelClass::initialize()
{
  rt_InitInfAndNaN(sizeof(real_T));
  (void) std::memset(static_cast<void *>(&fbw_DWork), 0,
                     sizeof(D_Work_fbw_T));
  (void)std::memset(&fbw_U, 0, sizeof(ExternalInputs_fbw_T));
  (void) std::memset(static_cast<void *>(&fbw_Y), 0,
                     sizeof(ExternalOutputs_fbw_T));
  fbw_DWork.PrevY = fbw_P.RateLimitereta_IC;
  fbw_DWork.PrevY_e = fbw_P.RateLimiter_IC;
  fbw_DWork.UD_DSTATE = fbw_P.PID_DifferentiatorICPrevScaledInput;
  fbw_DWork.DiscreteTransferFcn1_states = fbw_P.DiscreteTransferFcn1_InitialStates;
  fbw_DWork.PrevY_g[0] = fbw_P.Limiter_IC;
  fbw_DWork.PrevY_g[1] = fbw_P.Limiter_IC;
  fbw_DWork.PrevY_g[2] = fbw_P.Limiter_IC;
  fbw_DWork.Integration_IC_LOADING = 1U;
  fbw_DWork.Integration_PrevResetState = 2;
  fbw_DWork.PrevY_p = fbw_P.RateLimit_IC;
  fbw_DWork.PrevY_d = fbw_P.RateLimitereta_IC_c;
  fbw_DWork.Integration13s_DSTATE = fbw_P.Integration13s_IC;
  fbw_DWork.Integration13s_PrevResetState = 2;
  fbw_DWork.PrevY_n = fbw_P.RateLimiter03s_IC;
  fbw_DWork.PrevY_gj = fbw_P.RateLimit_IC_b;
  fbw_DWork.PrevY_c = fbw_P.RateLimiterxi_IC;
  fbw_DWork.DiscreteTimeIntegrator_IC_LOADING = 1U;
  fbw_DWork.DiscreteTimeIntegrator_PrevResetState = 2;
  fbw_DWork.PrevY_j = fbw_P.RateLimiterxi_IC_c;
  fbw_DWork.is_active_c1_fbw = 0U;
  fbw_DWork.is_c1_fbw = fbw_IN_NO_ACTIVE_CHILD;
  fbw_DWork.is_active_c3_fbw = 0U;
  fbw_DWork.is_c3_fbw = fbw_IN_NO_ACTIVE_CHILD;
  fbw_DWork.chartAbsoluteTimeCounter = 0;
  fbw_DWork.is_active_c5_fbw = 0U;
  fbw_DWork.is_c5_fbw = fbw_IN_NO_ACTIVE_CHILD;
}

void fbwModelClass::terminate()
{
}

fbwModelClass::fbwModelClass() : fbw_M()
{
}

fbwModelClass::~fbwModelClass()
{
}

RT_MODEL_fbw_T * fbwModelClass::getRTM()
{
  return (&fbw_M);
}
