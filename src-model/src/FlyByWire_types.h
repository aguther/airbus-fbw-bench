#ifndef RTW_HEADER_FlyByWire_types_h_
#define RTW_HEADER_FlyByWire_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_base_raw_data_
#define DEFINED_TYPEDEF_FOR_base_raw_data_

typedef struct {
  real_T nz_g;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T qk_rad_s;
  real_T rk_rad_s;
  real_T pk_rad_s;
  real_T Vk_kt;
  real_T radio_height_ft;
  real_T CG_percent_MAC;
} base_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_input_
#define DEFINED_TYPEDEF_FOR_base_input_

typedef struct {
  real_T delta_eta_pos;
  real_T delta_xi_pos;
} base_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_fbw_input_
#define DEFINED_TYPEDEF_FOR_fbw_input_

typedef struct {
  base_raw_data data;
  base_input input;
} fbw_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_output_
#define DEFINED_TYPEDEF_FOR_base_output_

typedef struct {
  real_T eta_pos;
  real_T iH_deg;
  real_T xi_pos;
} base_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_
#define DEFINED_TYPEDEF_FOR_base_raw_

typedef struct {
  base_raw_data data;
  base_input input;
  base_output output;
} base_raw;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_data_
#define DEFINED_TYPEDEF_FOR_base_data_

typedef struct {
  real_T nz_g;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T qk_deg_s;
  real_T rk_deg_s;
  real_T pk_deg_s;
  real_T Vk_kt;
  real_T radio_height_ft;
  real_T CG_percent_MAC;
} base_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_data_computed_
#define DEFINED_TYPEDEF_FOR_base_data_computed_

typedef struct {
  real_T on_ground;
} base_data_computed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_sim_
#define DEFINED_TYPEDEF_FOR_base_sim_

typedef struct {
  base_raw raw;
  base_data data;
  base_data_computed data_computed;
  base_input input;
} base_sim;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_data_computed_
#define DEFINED_TYPEDEF_FOR_base_pitch_data_computed_

typedef struct {
  real_T in_flight;
  real_T in_flare;
  real_T in_flight_gain;
} base_pitch_data_computed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_normal_
#define DEFINED_TYPEDEF_FOR_base_pitch_normal_

typedef struct {
  real_T Cstar_c_g;
  real_T Cstar_g;
  real_T eta_dot_pos_s;
} base_pitch_normal;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_law_output_
#define DEFINED_TYPEDEF_FOR_base_pitch_law_output_

typedef struct {
  real_T eta_dot_pos_s;
} base_pitch_law_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_protection_
#define DEFINED_TYPEDEF_FOR_base_pitch_protection_

typedef struct {
  base_pitch_law_output attitude_min;
  base_pitch_law_output attitude_max;
} base_pitch_protection;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_integrated_
#define DEFINED_TYPEDEF_FOR_base_pitch_integrated_

typedef struct {
  real_T eta_pos;
} base_pitch_integrated;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_output_
#define DEFINED_TYPEDEF_FOR_base_pitch_output_

typedef struct {
  real_T eta_pos;
  real_T iH_deg;
} base_pitch_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_pitch_
#define DEFINED_TYPEDEF_FOR_base_pitch_

typedef struct {
  base_pitch_data_computed data_computed;
  base_pitch_normal law_normal;
  base_pitch_protection law_protection;
  base_pitch_law_output vote;
  base_pitch_integrated integrated;
  base_pitch_output output;
} base_pitch;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_roll_data_computed_
#define DEFINED_TYPEDEF_FOR_base_roll_data_computed_

typedef struct {
  real_T in_flight;
  real_T in_flight_gain;
} base_roll_data_computed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_roll_normal_
#define DEFINED_TYPEDEF_FOR_base_roll_normal_

typedef struct {
  real_T pk_c_deg_s;
  real_T Phi_c_deg;
  real_T xi_pos;
} base_roll_normal;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_roll_output_
#define DEFINED_TYPEDEF_FOR_base_roll_output_

typedef struct {
  real_T xi_pos;
} base_roll_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_roll_
#define DEFINED_TYPEDEF_FOR_base_roll_

typedef struct {
  base_roll_data_computed data_computed;
  base_roll_normal law_normal;
  base_roll_output output;
} base_roll;

#endif

#ifndef DEFINED_TYPEDEF_FOR_fbw_output_
#define DEFINED_TYPEDEF_FOR_fbw_output_

typedef struct {
  base_sim sim;
  base_pitch pitch;
  base_roll roll;
} fbw_output;

#endif

#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH_FlyByWire_T
#define typedef_skA4KFEZ4HPkJJBOYCrevdH_FlyByWire_T

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH_FlyByWire_T;

#endif

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_FlyByWire_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_FlyByWire_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E_FlyByWire_T;

#endif

typedef struct Parameters_FlyByWire_T_ Parameters_FlyByWire_T;

#endif

