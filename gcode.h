/*
  gcode.h - rs274/ngc parser.

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _GCODE_H_
#define _GCODE_H_

#include "nuts_bolts.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "errors.h"

#define MAX_OFFSET_ENTRIES 4 // must be a power of 2

typedef uint32_t tool_id_t;
typedef uint16_t macro_id_t;
typedef int8_t offset_id_t;

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.
// NOTE: Some values are assigned specific values to make g-code state reporting and parsing
// compile a little smaller. Although not
// ideal, just be careful with values that state 'do not alter' and check both report.c and gcode.c
// to see how they are used, if you need to alter them.

/*! Modal Group G0: Non-modal actions

Do not alter values!
*/
typedef enum {
    NonModal_NoAction = 0,                  //!< 0 - Default, must be zero
    NonModal_Dwell = 4,                     //!< 4 - G4
    NonModal_SetCoordinateData = 10,        //!< 10 - G10
    NonModal_GoHome_0 = 28,                 //!< 28 - G28
    NonModal_SetHome_0 = 38,                //!< 38 - G28.1
    NonModal_GoHome_1 = 30,                 //!< 30 - G30
    NonModal_SetHome_1 = 40,                //!< 40 - G30.1
    NonModal_AbsoluteOverride = 53,         //!< 53 - G53
    NonModal_MacroCall = 65,                //!< 65 - G65
    NonModal_SetCoordinateOffset = 92,      //!< 92 - G92
    NonModal_ResetCoordinateOffset = 102,   //!< 102 - G92.1
    NonModal_ClearCoordinateOffset = 112,   //!< 112 - G92.2
 #if ENABLE_ACCELERATION_PROFILES
    NonModal_RestoreCoordinateOffset = 122, //!< 122 - G92.3
    NonModal_SetAccelerationProfile = 187   //!< 187 - G187 
 #else
    NonModal_RestoreCoordinateOffset = 122 //!< 122 - G92.3
 #endif
} non_modal_t;


typedef enum {
    ModalState_NoAction = 0,                //!< 0 - Default, must be zero
    ModalState_Save = 70,                   //!< 70 - M70
    ModalState_Invalidate = 71,             //!< 71 - M71
    ModalState_Restore = 72,                //!< 72 - M72
    ModalState_SaveAutoRestore = 73,        //!< 73 - M73
} modal_state_action_t;

/*! Modal Group G1: Motion modes

Do not alter values!
*/
typedef enum {
    MotionMode_Seek = 0,                    //!< 0 - G0 - Default, must be zero
    MotionMode_Linear = 1,                  //!< 1 - G1
    MotionMode_CwArc = 2,                   //!< 2 - G2
    MotionMode_CcwArc = 3,                  //!< 3 - G3
    MotionMode_CubicSpline = 5,             //!< 5 - G5
    MotionMode_QuadraticSpline = 51,        //!< 51 - G5.1
    MotionMode_SpindleSynchronized = 33,    //!< 33 - G33
    MotionMode_RigidTapping = 331,          //!< 331 - G33.1
    MotionMode_DrillChipBreak = 73,         //!< 73 - G73
    MotionMode_Threading = 76,              //!< 76 - G76
    MotionMode_CannedCycle81 = 81,          //!< 81 - G81
    MotionMode_CannedCycle82 = 82,          //!< 82 - G82
    MotionMode_CannedCycle83 = 83,          //!< 83 - G83
    MotionMode_CannedCycle85 = 85,          //!< 85 - G85
    MotionMode_CannedCycle86 = 86,          //!< 86 - G86
    MotionMode_CannedCycle89 = 89,          //!< 89 - G89
    MotionMode_ProbeToward = 140,           //!< 140 - G38.2
    MotionMode_ProbeTowardNoError = 141,    //!< 141 - G38.3
    MotionMode_ProbeAway = 142,             //!< 142 - G38.4
    MotionMode_ProbeAwayNoError = 143,      //!< 143 - G38.5
    MotionMode_None = 80                    //!< 80 - G80
} motion_mode_t;

/*! Modal Group G2: Plane select

Do not alter values!
*/
typedef enum {
    PlaneSelect_XY = 0, //!< 0 - G17 - Default, must be zero
    PlaneSelect_ZX = 1, //!< 1 - G18
    PlaneSelect_YZ = 2  //!< 2 - G19
} plane_select_t;

// Modal Group G4: Arc IJK distance mode
//#define DISTANCE_ARC_MODE_INCREMENTAL 0 // G91.1 - Default, must be zero


/*! Modal Group G5: Feed rate mode

Do not alter values!
*/
typedef enum {
    FeedMode_UnitsPerMin = 0,   //!< 0 - G94 - Default, must be zero
    FeedMode_InverseTime = 1,   //!< 1 - G93
    FeedMode_UnitsPerRev = 2    //!< 2 - G95
} feed_mode_t;

// Modal Group G7: Cutter radius compensation mode
//#define CUTTER_COMP_DISABLE 0 // G40 - Default, must be zero

/*! Modal Group G8: Tool length offset

Do not alter values!
*/
typedef enum {
    ToolLengthOffset_Cancel = 0,         //!< 0 - G49 - Default, must be zero
    ToolLengthOffset_Enable = 1,         //!< 1 - G43
    ToolLengthOffset_EnableDynamic = 2,  //!< 2 - G43.1
    ToolLengthOffset_ApplyAdditional = 3 //!< 3 - G43.2
} tool_offset_mode_t;

/*! Modal Group G10: Canned cycle return mode

Do not alter values!
*/
typedef enum {
    CCRetractMode_Previous = 0,  //!< 0 - G98 - Default, must be zero
    CCRetractMode_RPos = 1       //!< 1 - G99
} cc_retract_mode_t;


/*! Modal Group G12 and G0: Coordinate system identificators

Do not alter values!
*/

typedef enum  {
    CoordinateSystem_G54 = 0,                       //!< 0 - G54 (G12)
    CoordinateSystem_G55,                           //!< 1 - G55 (G12)
    CoordinateSystem_G56,                           //!< 2 - G56 (G12)
    CoordinateSystem_G57,                           //!< 3 - G57 (G12)
    CoordinateSystem_G58,                           //!< 4 - G58 (G12)
    CoordinateSystem_G59,                           //!< 5 - G59 (G12)
#if COMPATIBILITY_LEVEL <= 1
    CoordinateSystem_G59_1,                         //!< 6 - G59.1 (G12) - availability depending on #COMPATIBILITY_LEVEL <= 1
    CoordinateSystem_G59_2,                         //!< 7 - G59.2 (G12) - availability depending on #COMPATIBILITY_LEVEL <= 1
    CoordinateSystem_G59_3,                         //!< 8 - G59.3 (G12) - availability depending on #COMPATIBILITY_LEVEL <= 1
#endif
    N_WorkCoordinateSystems,                        //!< 9 when #COMPATIBILITY_LEVEL <= 1, 6 otherwise
    CoordinateSystem_G28 = N_WorkCoordinateSystems, //!< 9 - G28 (G0) when #COMPATIBILITY_LEVEL <= 1, 6 otherwise
    CoordinateSystem_G30,                           //!< 10 - G30 (G0) when #COMPATIBILITY_LEVEL <= 1, 7 otherwise
    CoordinateSystem_G92,                           //!< 11 - G92 (G0) when #COMPATIBILITY_LEVEL <= 1, 8 otherwise
    N_CoordinateSystems                             //!< 12 when #COMPATIBILITY_LEVEL <= 1, 9 otherwise
}  __attribute__ ((__packed__)) coord_system_id_t;


/*!  Modal Group G13: Control mode

Do not alter values!
*/
typedef enum {
    ControlMode_ExactPath = 0,      //!< 0 - G61 - Default, must be zero
    ControlMode_ExactStop = 1,      //!< 1 - G61.1
    ControlMode_PathBlending = 2    //!< 2 - G64
} control_mode_t;

/*!  Modal Group G14: Spindle Speed Mode

Do not alter values!
*/
typedef enum {
    SpindleSpeedMode_RPM = 0,  //!< 0 - G97 - Default, must be zero
    SpindleSpeedMode_CSS = 1   //!< 1 - G96
} spindle_rpm_mode_t;

/*! Modal Group M4: Program flow

Do not alter values!
*/
typedef enum {
    ProgramFlow_Running = 0,        //!< 0  - Default, must be zero
    ProgramFlow_Paused  = 3,        //!< 3 - M0
    ProgramFlow_OptionalStop = 1,   //!< 1 - M1
    ProgramFlow_CompletedM2 = 2,    //!< 2 - M2
    ProgramFlow_CompletedM30 = 30,  //!< 30 - M30
    ProgramFlow_CompletedM60 = 60,  //!< 60 - M60
    ProgramFlow_Return = 99         //!< 99 - M99
} program_flow_t;

// Modal Group M9: Override control
typedef enum {
    Override_FeedSpeedEnable = 48,  //!< 48 - M48
    Override_FeedSpeedDisable = 49, //!< 49 - M49
    Override_FeedRate = 50,         //!< 50 - M50
    Override_SpindleSpeed = 51,     //!< 51 - M51
    Override_FeedHold = 53,         //!< 53 - M53
    Override_Parking = 56           //!< 56 - M56
} override_mode_t;

/*! Modal Group M10: i/o control */

typedef enum {
    IoMCode_OutputOnSynced = 62,        //!< 62 - M62
    IoMCode_OutputOffSynced = 63,       //!< 63 - M63
    IoMCode_OutputOnImmediate = 64,     //!< 64 - M64
    IoMCode_OutputOffImmediate = 65,    //!< 65 - M65
    IoMCode_WaitOnInput = 66,           //!< 66 - M66
    IoMCode_AnalogOutSynced = 67,       //!< 67 - M67
    IoMCode_AnalogOutImmediate = 68,    //!< 68 - M68
} io_mcode_t;

/*! Modal Group M10: User/driver/plugin defined M commands
__NOTE:__ Not used by the core, may be used by private user code, drivers or plugins.
*/
typedef enum {
    OpenPNP_SetPinState = 42,           //!< 42 - M42
    UserMCode_Generic1 = 101,           //!< 101 - For private use only
    UserMCode_Generic2 = 102,           //!< 102 - For private use only
    UserMCode_Generic3 = 103,           //!< 103 - For private use only
    UserMCode_Generic4 = 104,           //!< 104 - For private use only
    OpenPNP_GetADCReading = 105,        //!< 105 - M105
    Fan_On = 106,                       //!< 106 - M106, Marlin format
    Fan_Off = 107,                      //!< 107 - M107, Marlin format
    OpenPNP_GetCurrentPosition = 114,   //!< 114 - M114
    OpenPNP_FirmwareInfo = 115,         //!< 115 - M115
    Trinamic_DebugReport = 122,         //!< 122 - M122, Marlin format
    Trinamic_ReadRegister = 123,        //!< 123 - M123
    Trinamic_WriteRegister = 124,       //!< 124 - M124
    LaserPPI_Enable = 126,              //!< 126 - M126
    LaserPPI_Rate = 127,                //!< 127 - M127
    LaserPPI_PulseLength = 128,         //!< 128 - M128
    RGB_WriteLEDs = 150,                //!< 150 - M150, Marlin format
    OpenPNP_SetAcceleration = 204,      //!< 204 - M204
    SetFeedOverrides = 220,             //!< 220 - M220, Marlin format
    PWMServo_SetPosition= 280,          //!< 280 - M280, Marlin format
    RGB_Inspection_Light = 356,         //!< 356 - M356
    OpenPNP_FinishMoves = 400,          //!< 400 - M400
    Probe_Deploy = 401,                 //!< 401 - M401, Marlin format
    Probe_Stow = 402,                   //!< 402 - M402, Marlin format
    OpenPNP_SettingsReset = 502,        //!< 502 - M502
    Trinamic_ModeToggle = 569,          //!< 569 - M569, Marlin format
    Trinamic_StepperCurrent = 906,      //!< 906 - M906, Marlin format
    Trinamic_ReportPrewarnFlags = 911,  //!< 911 - M911, Marlin format
    Trinamic_ClearPrewarnFlags = 912,   //!< 912 - M912, Marlin format
    Trinamic_HybridThreshold = 913,     //!< 913 - M913, Marlin format
    Trinamic_HomingSensitivity = 914,   //!< 914 - M914, Marlin format
    Trinamic_ChopperTiming = 919,       //!< 919 - M919, Marlin format
    Spindle_Select = UserMCode_Generic4 //!< Value to be assigned later!
} user_mcode_t;

//! Data for M62, M63 and M67 commands when executed synchronized with motion.
typedef struct output_command {
    bool is_digital;
    bool is_executed;
    uint8_t port;
    int32_t value;
    struct output_command *next;
} output_command_t;

//! M66 Allowed L-parameter values
typedef enum {
    WaitMode_Immediate = 0, //!< 0 - This is the only mode allowed for analog inputs
    WaitMode_Rise,          //!< 1
    WaitMode_Fall,          //!< 2
    WaitMode_High,          //!< 3
    WaitMode_Low,           //!< 4
    WaitMode_Max            //!< For internal use
} wait_mode_t;

//! Parser position updating flags
typedef enum {
    GCUpdatePos_Target = 0, //!< 0
    GCUpdatePos_System,     //!< 1
    GCUpdatePos_None        //!< 2
} pos_update_t;

/*! Probe cycle exit states, used for proper proper position updating.

Assigned from #pos_update_t enum values.
*/
typedef enum {
    GCProbe_Found = GCUpdatePos_System,     //!< 1
    GCProbe_Abort = GCUpdatePos_None,       //!< 2
    GCProbe_FailInit = GCUpdatePos_None,    //!< 2
    GCProbe_FailEnd = GCUpdatePos_Target,   //!< 0
  #if SET_CHECK_MODE_PROBE_TO_START
    GCProbe_CheckMode = GCUpdatePos_None    //!< 2
  #else
    GCProbe_CheckMode = GCUpdatePos_Target  //!< 0
  #endif
} gc_probe_t;

//! Parser flags for special cases.
typedef union {
    uint16_t value;
    struct {
        uint16_t jog_motion          :1,
                 canned_cycle_change :1, // Use motion_mode_changed?
                 arc_is_clockwise    :1,
                 probe_is_away       :1,
                 probe_is_no_error   :1,
                 spindle_force_sync  :1,
                 laser_disable       :1,
                 laser_is_motion     :1,
                 set_coolant         :1,
                 motion_mode_changed :1,
                 reserved            :6;
    };
} gc_parser_flags_t;

//! Override flags.
typedef union {
    uint8_t value;
    struct {
        uint8_t feed_rate_disable   :1,
                feed_hold_disable   :1,
                spindle_rpm_disable :1,
                parking_disable     :1,
                reserved            :3,
                sync                :1;
    };
} gc_override_flags_t;

//! Coordinate data.
typedef union {
    float values[N_AXIS];
    struct {
        float x;
        float y;
        float z;
#ifdef A_AXIS
        float a;
#endif
#ifdef B_AXIS
        float b;
#endif
#ifdef C_AXIS
        float c;
#endif
#ifdef U_AXIS
        float u;
#endif
#ifdef V_AXIS
        float v;
#endif
    };
    struct {
        float m0;
        float m1;
        float m2;
#if N_AXIS > 3
        float m3;
#endif
#if N_AXIS > 4
        float m4;
#endif
#if N_AXIS > 5
        float m5;
#endif
#if N_AXIS > 6
        float m6;
#endif
#if N_AXIS == 8
        float m7;
#endif
    };
} coord_data_t;

//! Coordinate data including id.
typedef struct {
    float xyz[N_AXIS];
    coord_system_id_t id;
} coord_system_t;

//! Axis index to plane assignment.
typedef union {
    uint8_t axis[3];
    struct {
        uint8_t axis_0;
        uint8_t axis_1;
        uint8_t axis_linear;
    };
} plane_t;

/*! \brief G- and M-code parameter values

After the parameters in a block is parsed into the parser blocks (parser_block_t) \a values its
corresponding \a words (#parameter_words_t) union holds which parameters were found.

__NOTE:__ Do not use single-meaning words in user defined M-codes.
*/
typedef struct {
    float d;                   //!< Max spindle RPM in Constant Surface Speed Mode (G96)
    float e;                   //!< Thread taper length (G76), M67 output number
    float f;                   //!< Feed rate - single-meaning word
    float ijk[3];              //!< I,J,K Axis arc offsets
    float k;                   //!< G33 distance per revolution
    float m;                   //!< G65 argument.
    float p;                   //!< G10, 664 or dwell parameters
    float q;                   //!< User defined M-code parameter, M67 output value, G64 naive CAM tolerance, G83 delta increment
    float r;                   //!< Arc radius or retract position
    float s;                   //!< Spindle speed - single-meaning word
#ifndef A_AXIS
    float a;
#endif
#ifndef B_AXIS
    float b;
#endif
#ifndef C_AXIS
    float c;
#endif
#if !defined(U_AXIS) && !AXIS_REMAP_ABC2UVW
    float u;
#endif
#if !defined(V_AXIS) && !AXIS_REMAP_ABC2UVW
    float v;
#endif
#if !AXIS_REMAP_ABC2UVW
    float w;
#endif
    float xyz[N_AXIS];         //!< X,Y,Z (and A,B,C,U,V when enabled) translational axes
#if LATHE_UVW_OPTION
    float uvw[3];              //!< U,V,W lathe mode incremental mode motion
#endif
    coord_system_t coord_data; //!< Coordinate data
    int32_t $;                 //!< Spindle id - single-meaning word
    int32_t n;                 //!< Line number - single-meaning word
    uint32_t o;                //!< Subroutine identifier - single-meaning word
    uint32_t h;                //!< Tool number or number of G76 thread spring passes
    tool_id_t t;               //!< Tool selection - single-meaning word
    uint8_t l;                 //!< G10 or canned cycles parameters
} gc_values_t;

//! Parameter words found by parser - do not change order!
typedef union {
    uint32_t mask;      //!< All flags as a bitmap.
    uint32_t value;     //!< Synonymous with \a mask.
    struct {
        uint32_t $ :1, //!< Spindle id.
                 a :1, //!< A-axis.
                 b :1, //!< B-axis.
                 c :1, //!< C-axis.
                 i :1, //!< X-axis offset for arcs.
                 j :1, //!< Y-axis offset for arcs.
                 k :1, //!< Z-axis offset for arcs.
                 d :1, //!< Tool radius compensation.
                 e :1, //!< Analog port number for M66 - M68.
                 f :1, //!< Feedrate.
                 g :1, //!< Unused (placeholder).
                 h :1, //!< Tool length offset index.
                 l :1, //!< Number of repetitions in canned cycles, wait mode for M66.
                 m :1, //!< G65 argument.
                 n :1, //!< Line number.
                 o :1, //!< Subroutine identifier.
                 p :1, //!< Dwell time for G4 or in canned cycles, port number for M62 - M66.
                 q :1, //!< Feed increment for G83 canned cycle, tool number for M61, timeout for M66.
                 r :1, //!< Arc radius, canned cycle retract level.
                 s :1, //!< Spindle speed.
                 t :1, //!< Tool number.
                 u :1, //!< U-axis.
                 v :1, //!< V-axis.
                 w :1, //!< W-axis.
                 x :1, //!< X-axis.
                 y :1, //!< Y-axis.
                 z :1; //!< Z-axis.
    };
} parameter_words_t;

typedef enum {
    ValueType_NA = 0,
    ValueType_UInt8,
    ValueType_UInt32,
    ValueType_Int32,
    ValueType_Float
} gc_value_type_t;

typedef struct {
    const void *value;
    const gc_value_type_t type;
} gc_value_ptr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t is_rpm_rate_adjusted :1,
                is_laser_ppi_mode    :1,
                unassigned           :6;

    };
} spindle_cond_t;

typedef struct {
    spindle_state_t state;          //!< {M3,M4,M5}
    spindle_rpm_mode_t rpm_mode;    //!< {G96,G97}
    spindle_css_data_t *css;        //!< Data used for Constant Surface Speed Mode calculations
    spindle_cond_t condition;       //!< TODO: move data from planner_cond_t here
    float rpm;                      //!< Spindle speed. Must be second last!
    spindle_ptrs_t *hal;            //!< Spindle function pointers etc. Must be last!
} spindle_t;

typedef struct {
    spindle_state_t state;          //!< {M3,M4,M5}
    spindle_rpm_mode_t rpm_mode;    //!< {G96,G97}
} spindle_modal_t;

// NOTE: When this struct is zeroed, the above defines set the defaults for the system.
typedef struct {
    motion_mode_t motion;                //!< {G0,G1,G2,G3,G38.2,G80}
    feed_mode_t feed_mode;               //!< {G93,G94,G95}
    bool units_imperial;                 //!< {G20,G21}
    bool distance_incremental;           //!< {G90,G91}
    bool diameter_mode;                  //!< {G7,G8} Lathe diameter mode.
    //< uint8_t distance_arc;            //!< {G91.1} NOTE: Don't track. Only default supported.
    plane_select_t plane_select;         //!< {G17,G18,G19}
    //< uint8_t cutter_comp;             //!< {G40} NOTE: Don't track. Only default supported.
    tool_offset_mode_t tool_offset_mode; //!< {G43,G43.1,G49}
    coord_system_t coord_system;         //!< {G54,G55,G56,G57,G58,G59,G59.1,G59.2,G59.3}
#if ENABLE_PATH_BLENDING
    control_mode_t control;              //!< {G61} NOTE: Don't track. Only default supported.
#endif
    program_flow_t program_flow;         //!< {M0,M1,M2,M30,M60}
    coolant_state_t coolant;             //!< {M7,M8,M9}
#if N_SYS_SPINDLE > 1
    spindle_t spindle[N_SYS_SPINDLE];
#else
    spindle_t spindle;                   //!< {M3,M4,M5 and G96,G97}
#endif
    gc_override_flags_t override_ctrl;   //!< {M48,M49,M50,M51,M53,M56}
    cc_retract_mode_t retract_mode;      //!< {G98,G99}
    bool scaling_active;                 //!< {G50,G51}
    bool canned_cycle_active;
    float spline_pq[2];                  //!< {G5}
#if NGC_PARAMETERS_ENABLE
    bool auto_restore;
    float feed_rate;                     //!< {F} NOTE: only set when saving modal state
#endif
#if ENABLE_ACCELERATION_PROFILES
    float acceleration_factor;          //!< {G187} currently active factor of acceleration profile
#endif
} gc_modal_t;

//! Data for canned cycles.
typedef struct {
    float xyz[3];
    float delta;
    float dwell;
    float retract_position; //!< Canned cycle retract position
    bool rapid_retract;
    bool spindle_off;
    cc_retract_mode_t retract_mode;
    bool change;
} gc_canned_t;

//! Thread taper types.
typedef enum {
    Taper_None = 0, //!< 0
    Taper_Entry,    //!< 1
    Taper_Exit,     //!< 2
    Taper_Both      //!< 3
} gc_taper_type;

typedef struct {
    float pitch;
    float z_final;
    float peak;
    float initial_depth;
    float depth;
    float depth_degression;
    float main_taper_height;
    float end_taper_length;
    float infeed_angle;
    float cut_direction;
    uint_fast16_t spring_passes;
    gc_taper_type end_taper_type;
} gc_thread_data;

//! Tool data.
typedef struct {
    float offset[N_AXIS];   //!< Tool offset
    float radius;           //!< Radius of tool (currently unsupported)
// TODO: add float max_rpm; ?
    tool_id_t tool_id;      //!< Tool number
} tool_data_t;

/*! \brief Parser state

*/
typedef struct {
    gc_modal_t modal;
    gc_canned_t canned;
    spindle_t *spindle;                 //!< Last referenced spindle
    float feed_rate;                    //!< Millimeters/min
    float distance_per_rev;             //!< Millimeters/rev
    float position[N_AXIS];             //!< Where the interpreter considers the tool to be at this point in the code
#if ENABLE_PATH_BLENDING
    float path_tolerance;               //!< Path blending tolerance
    float cam_tolerance;                //!< Naive CAM tolerance
#endif
    int32_t line_number;                //!< Last line number sent
    tool_id_t tool_pending;             //!< Tool to be selected on next M6
#if NGC_EXPRESSIONS_ENABLE
    uint32_t g43_pending;               //!< Tool offset to be selected on next M6, for macro ATC
#endif
    bool file_run;                      //!< Tracks % command
    bool is_laser_ppi_mode;
    bool is_rpm_rate_adjusted;
    bool tool_change;
    bool skip_blocks;                   //!< true if skipping conditional blocks
    status_code_t last_error;           //!< last return value from parser
    offset_id_t offset_id;              //!< id(x) of last G92 coordinate offset (into circular buffer)
    coord_data_t offset_queue[MAX_OFFSET_ENTRIES];
    //!< The following variables are not cleared upon warm restart when COMPATIBILITY_LEVEL <= 1
    bool g92_coord_offset_applied;      //!< true when G92 offset applied
    float g92_coord_offset[N_AXIS];     //!< Retains the G92 coordinate offset (work coordinates) relative to
                                        //!< machine zero in mm. Persistent and loaded from non-volatile storage
                                        //!< on boot when COMPATIBILITY_LEVEL <= 1
    float tool_length_offset[N_AXIS];   //!< Tracks tool length offset when enabled
    tool_data_t *tool;                  //!< Tracks tool number and tool offset
} parser_state_t;

typedef struct {
    float xyz[N_AXIS]; //!< Center point
    float ijk[N_AXIS]; //!< Scaling factors
} scale_factor_t;

extern parser_state_t gc_state;

/*! \brief Parser block structure.

Used internally by the parser to hold the details about a block.
It will also be passed to mc_jog_execute() and any user M-code validation and execution handlers if called for.
 */
typedef struct {
    non_modal_t non_modal_command;      //!< Non modal command
    override_mode_t override_command;   //!< Override command TODO: add to non_modal above?
    user_mcode_t user_mcode;            //!< Set > 0 if a user M-code is found.
    bool user_mcode_sync;               //!< Set to \a true by M-code validation handler if M-code is to be executed after synchronization.
    gc_modal_t modal;                   //!< The current modal state is copied here before parsing starts.
    spindle_modal_t spindle_modal;
    gc_values_t values;                 //!< Parameter values for block.
    parameter_words_t words;            //!< Bitfield for tracking found parameter values.
    output_command_t output_command;    //!< Details about M62-M68 output command to execute if present in block.
    uint32_t arc_turns;                 //
#if NGC_PARAMETERS_ENABLE
    modal_state_action_t state_action;  //!< M70-M73 modal state action
#endif
#if N_AXIS > 3
    axes_signals_t rotary_wrap;
#endif
} parser_block_t;

// Initialize the parser
void gc_init (bool stop);

char *gc_normalize_block (char *block, status_code_t *status, char **message);

// Execute one block of rs275/ngc/g-code
status_code_t gc_execute_block (char *block);

// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
#define gc_sync_position() system_convert_array_steps_to_mpos (gc_state.position, sys.position)

// Sets g-code parser and planner position in mm.
#define sync_position() plan_sync_position(); system_convert_array_steps_to_mpos (gc_state.position, sys.position)

// Set dynamic laser power mode to PPI (Pulses Per Inch)
// Driver support for pulsing the laser on signal is required for this to work.
// Returns true if driver uses hardware implementation.
bool gc_laser_ppi_enable (uint_fast16_t ppi, uint_fast16_t pulse_length);
parser_state_t *gc_get_state (void);
// Gets axes scaling state.
axes_signals_t gc_get_g51_state (void);
float *gc_get_scaling (void);

// Get current axis offset.
float gc_get_offset (uint_fast8_t idx, bool real_time);

spindle_t *gc_spindle_get (spindle_num_t spindle);

void gc_spindle_off (void);
void gc_coolant (coolant_state_t state);

void gc_set_tool_offset (tool_offset_mode_t mode, uint_fast8_t idx, int32_t offset);
plane_t *gc_get_plane_data (plane_t *plane, plane_select_t select);

#if NGC_PARAMETERS_ENABLE
parameter_words_t gc_get_g65_arguments (void);
bool gc_modal_state_restore (gc_modal_t *copy);
#endif

#if ENABLE_ACCELERATION_PROFILES
float gc_get_accel_factor (uint8_t profile);
#endif

#endif // _GCODE_H_
