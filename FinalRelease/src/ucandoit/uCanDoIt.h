//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: uCanDoIt.h
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
#ifndef RTW_HEADER_uCanDoIt_h_
#define RTW_HEADER_uCanDoIt_h_
#include <stddef.h>
#include <string.h>
#ifndef uCanDoIt_COMMON_INCLUDES_
# define uCanDoIt_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // uCanDoIt_COMMON_INCLUDES_

#include "uCanDoIt_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  SL_Bus_uCanDoIt_geometry_msgs_Twist In1;// '<S5>/In1'
  boolean_T stopped;                   // '<Root>/Chart'
} B_uCanDoIt_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  void *SourceBlock_PWORK;             // '<S4>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S3>/SinkBlock'
  robotics_slros_internal_block_T obj; // '<S3>/SinkBlock'
  robotics_slros_internal_blo_c_T obj_n;// '<S4>/SourceBlock'
  uint8_T is_active_c3_uCanDoIt;       // '<Root>/Chart'
  uint8_T is_c3_uCanDoIt;              // '<Root>/Chart'
} DW_uCanDoIt_T;

// Parameters (auto storage)
struct P_uCanDoIt_T_ {
  SL_Bus_uCanDoIt_geometry_msgs_Twist Out1_Y0;// Computed Parameter: Out1_Y0
                                              //  Referenced by: '<S5>/Out1'

  SL_Bus_uCanDoIt_geometry_msgs_Twist Constant_Value;// Computed Parameter: Constant_Value
                                                     //  Referenced by: '<S4>/Constant'

  SL_Bus_uCanDoIt_std_msgs_Bool Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                 //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_uCanDoIt_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_uCanDoIt_T uCanDoIt_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_uCanDoIt_T uCanDoIt_B;

// Block states (auto storage)
extern DW_uCanDoIt_T uCanDoIt_DW;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void uCanDoIt_initialize(void);
  extern void uCanDoIt_step(void);
  extern void uCanDoIt_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_uCanDoIt_T *const uCanDoIt_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Data Type Conversion' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'uCanDoIt'
//  '<S1>'   : 'uCanDoIt/Blank Message'
//  '<S2>'   : 'uCanDoIt/Chart'
//  '<S3>'   : 'uCanDoIt/Publish'
//  '<S4>'   : 'uCanDoIt/Subscribe'
//  '<S5>'   : 'uCanDoIt/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_uCanDoIt_h_

//
// File trailer for generated code.
//
// [EOF]
//
