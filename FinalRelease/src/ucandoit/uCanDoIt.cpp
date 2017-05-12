//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: uCanDoIt.cpp
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
#include "uCanDoIt.h"
#include "uCanDoIt_private.h"

// Named constants for Chart: '<Root>/Chart'
#define uCanDoIt_IN_Idle               ((uint8_T)1U)
#define uCanDoIt_IN_Moving             ((uint8_T)2U)
#define uCanDoIt_IN_NO_ACTIVE_CHILD    ((uint8_T)0U)
#define uCanDoIt_IN_Stopped            ((uint8_T)3U)
#define uCanDoIt_MessageQueueLen       (1)

// Block signals (auto storage)
B_uCanDoIt_T uCanDoIt_B;

// Block states (auto storage)
DW_uCanDoIt_T uCanDoIt_DW;

// Real-time model
RT_MODEL_uCanDoIt_T uCanDoIt_M_;
RT_MODEL_uCanDoIt_T *const uCanDoIt_M = &uCanDoIt_M_;

// Model step function
void uCanDoIt_step(void)
{
  SL_Bus_uCanDoIt_geometry_msgs_Twist varargout_2;
  boolean_T varargout_1;
  SL_Bus_uCanDoIt_std_msgs_Bool rtb_BusAssignment;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S5>/In1'
  //   MATLABSystem: '<S4>/SourceBlock'

  varargout_1 = Sub_uCanDoIt_1.getLatestMessage(&varargout_2);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  if (varargout_1) {
    uCanDoIt_B.In1 = varargout_2;
  }

  // End of Start for MATLABSystem: '<S4>/SourceBlock'
  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Chart: '<Root>/Chart'
  // Gateway: Chart
  // During: Chart
  if (uCanDoIt_DW.is_active_c3_uCanDoIt == 0U) {
    // Entry: Chart
    uCanDoIt_DW.is_active_c3_uCanDoIt = 1U;

    // Entry Internal: Chart
    // Transition: '<S2>:5'
    uCanDoIt_DW.is_c3_uCanDoIt = uCanDoIt_IN_Idle;
  } else {
    switch (uCanDoIt_DW.is_c3_uCanDoIt) {
     case uCanDoIt_IN_Idle:
      // During 'Idle': '<S2>:4'
      // '<S2>:7:1' sf_internal_predicateOutput = ...
      // '<S2>:7:1' vel > 0.5;
      if (uCanDoIt_B.In1.Linear.X > 0.5) {
        // Transition: '<S2>:7'
        uCanDoIt_DW.is_c3_uCanDoIt = uCanDoIt_IN_Moving;
      } else {
        // '<S2>:4:1' stopped = false;
        uCanDoIt_B.stopped = false;
      }
      break;

     case uCanDoIt_IN_Moving:
      // During 'Moving': '<S2>:6'
      // '<S2>:9:1' sf_internal_predicateOutput = ...
      // '<S2>:9:1' vel < 0.01;
      if (uCanDoIt_B.In1.Linear.X < 0.01) {
        // Transition: '<S2>:9'
        uCanDoIt_DW.is_c3_uCanDoIt = uCanDoIt_IN_Stopped;
      } else {
        // '<S2>:6:1' stopped = false;
        uCanDoIt_B.stopped = false;
      }
      break;

     default:
      // During 'Stopped': '<S2>:8'
      // '<S2>:10:1' sf_internal_predicateOutput = ...
      // '<S2>:10:1' vel > 0.5;
      if (uCanDoIt_B.In1.Linear.X > 0.5) {
        // Transition: '<S2>:10'
        uCanDoIt_DW.is_c3_uCanDoIt = uCanDoIt_IN_Moving;
      } else {
        // '<S2>:8:1' stopped = true;
        uCanDoIt_B.stopped = true;
      }
      break;
    }
  }

  // End of Chart: '<Root>/Chart'

  // BusAssignment: '<Root>/Bus Assignment'
  rtb_BusAssignment.Data = uCanDoIt_B.stopped;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // Start for MATLABSystem: '<S3>/SinkBlock' incorporates:
  //   MATLABSystem: '<S3>/SinkBlock'

  Pub_uCanDoIt_79.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void uCanDoIt_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(uCanDoIt_M, (NULL));

  // block I/O
  (void) memset(((void *) &uCanDoIt_B), 0,
                sizeof(B_uCanDoIt_T));

  // states (dwork)
  (void) memset((void *)&uCanDoIt_DW, 0,
                sizeof(DW_uCanDoIt_T));

  {
    static const char_T tmp[5] = { '/', 'd', 'o', 'I', 't' };

    static const char_T tmp_0[15] = { '/', 'c', 'a', 't', 'v', 'e', 'h', 'i',
      'c', 'l', 'e', '/', 'v', 'e', 'l' };

    char_T tmp_1[6];
    char_T tmp_2[16];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S4>/SourceBlock'
    uCanDoIt_DW.obj_n.isInitialized = 0;
    uCanDoIt_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[15] = '\x00';
    Sub_uCanDoIt_1.createSubscriber(tmp_2, uCanDoIt_MessageQueueLen);

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    uCanDoIt_DW.obj.isInitialized = 0;
    uCanDoIt_DW.obj.isInitialized = 1;
    for (i = 0; i < 5; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[5] = '\x00';
    Pub_uCanDoIt_79.createPublisher(tmp_1, uCanDoIt_MessageQueueLen);

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S5>/Out1'
    uCanDoIt_B.In1 = uCanDoIt_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Chart: '<Root>/Chart'
    uCanDoIt_DW.is_active_c3_uCanDoIt = 0U;
    uCanDoIt_DW.is_c3_uCanDoIt = uCanDoIt_IN_NO_ACTIVE_CHILD;
  }
}

// Model terminate function
void uCanDoIt_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S4>/SourceBlock'

  if (uCanDoIt_DW.obj_n.isInitialized == 1) {
    uCanDoIt_DW.obj_n.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S4>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Start for MATLABSystem: '<S3>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S3>/SinkBlock'

  if (uCanDoIt_DW.obj.isInitialized == 1) {
    uCanDoIt_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S3>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
