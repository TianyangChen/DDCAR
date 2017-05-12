//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: uCanDoIt_types.h
//
// Code generated for Simulink model 'uCanDoIt'.
//
// Model version                  : 1.46
// Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
// C/C++ source code generated on : Sun Apr 23 16:07:57 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_uCanDoIt_types_h_
#define RTW_HEADER_uCanDoIt_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_uCanDoIt_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_std_msgs_Bool_

// MsgType=std_msgs/Bool
typedef struct {
  boolean_T Data;
} SL_Bus_uCanDoIt_std_msgs_Bool;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_uCanDoIt_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_uCanDoIt_geometry_msgs_Twist_

// MsgType=geometry_msgs/Twist
typedef struct {
  // MsgType=geometry_msgs/Vector3
  SL_Bus_uCanDoIt_geometry_msgs_Vector3 Linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_uCanDoIt_geometry_msgs_Vector3 Angular;
} SL_Bus_uCanDoIt_geometry_msgs_Twist;

#endif

#ifndef struct_tag_saG5948SFTrUFnIbVUb0TZH
#define struct_tag_saG5948SFTrUFnIbVUb0TZH

struct tag_saG5948SFTrUFnIbVUb0TZH
{
  int32_T isInitialized;
};

#endif                                 //struct_tag_saG5948SFTrUFnIbVUb0TZH

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct tag_saG5948SFTrUFnIbVUb0TZH robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef struct_tag_saG5948SFTrUFnIbVUb0TZH
#define struct_tag_saG5948SFTrUFnIbVUb0TZH

struct tag_saG5948SFTrUFnIbVUb0TZH
{
  int32_T isInitialized;
};

#endif                                 //struct_tag_saG5948SFTrUFnIbVUb0TZH

#ifndef typedef_robotics_slros_internal_blo_c_T
#define typedef_robotics_slros_internal_blo_c_T

typedef struct tag_saG5948SFTrUFnIbVUb0TZH robotics_slros_internal_blo_c_T;

#endif                                 //typedef_robotics_slros_internal_blo_c_T

#ifndef typedef_struct_T_uCanDoIt_T
#define typedef_struct_T_uCanDoIt_T

typedef struct {
  char_T f0[4];
} struct_T_uCanDoIt_T;

#endif                                 //typedef_struct_T_uCanDoIt_T

#ifndef typedef_struct_T_uCanDoIt_c_T
#define typedef_struct_T_uCanDoIt_c_T

typedef struct {
  char_T f0[8];
} struct_T_uCanDoIt_c_T;

#endif                                 //typedef_struct_T_uCanDoIt_c_T

#ifndef typedef_struct_T_uCanDoIt_cz_T
#define typedef_struct_T_uCanDoIt_cz_T

typedef struct {
  char_T f0[7];
} struct_T_uCanDoIt_cz_T;

#endif                                 //typedef_struct_T_uCanDoIt_cz_T

#ifndef typedef_struct_T_uCanDoIt_czh_T
#define typedef_struct_T_uCanDoIt_czh_T

typedef struct {
  char_T f0[8];
  char_T f1[7];
  char_T f2[6];
} struct_T_uCanDoIt_czh_T;

#endif                                 //typedef_struct_T_uCanDoIt_czh_T

// Parameters (auto storage)
typedef struct P_uCanDoIt_T_ P_uCanDoIt_T;

// Forward declaration for rtModel
typedef struct tag_RTM_uCanDoIt_T RT_MODEL_uCanDoIt_T;

#endif                                 // RTW_HEADER_uCanDoIt_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
