/*
  settings.h - non-volatile storage configuration handling

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
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

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include "config.h"
#include "system.h"
#include "plugins.h"

// Version of the persistent storage data. Always stored in byte 0 of non-volatile storage.
#define SETTINGS_VERSION 23  // NOTE: Check settings_reset() when moving to next version.

// Define axis settings numbering scheme. Starts at Setting_AxisSettingsBase, every INCREMENT, over N_SETTINGS.
#define AXIS_SETTINGS_INCREMENT  10 // Must be greater than the number of axis settings.

// Define encoder settings numbering scheme. Starts at Setting_EncoderSettingsBase, every INCREMENT, over N_SETTINGS.
// Not referenced by the core.
#define ENCODER_N_SETTINGS_MAX 5 // NOTE: This is the maximum number of encoders allowed.
#define ENCODER_SETTINGS_INCREMENT 10

#define SETTINGS_HARD_RESET_REQUIRED "\\n\\nNOTE: A hard reset of the controller is required after changing this setting."

#define PASSWORD_MASK "********"

typedef enum {
    Setting_PulseMicroseconds = 0,
    Setting_StepperIdleLockTime = 1,
    Setting_StepInvertMask = 2,
    Setting_DirInvertMask = 3,
    Setting_InvertStepperEnable = 4,
    Setting_LimitPinsInvertMask = 5,
    Setting_InvertProbePin = 6,
    Setting_SpindlePWMBehaviour = 7, // Deprecated - replaced by Setting_SpindlePWMOptions flag
    Setting_GangedDirInvertMask = 8,
    Setting_SpindlePWMOptions = 9,
    Setting_StatusReportMask = 10,
    Setting_JunctionDeviation = 11,
    Setting_ArcTolerance = 12,
    Setting_ReportInches = 13,
    Setting_ControlInvertMask = 14,
    Setting_CoolantInvertMask = 15,
    Setting_SpindleInvertMask = 16,
    Setting_ControlPullUpDisableMask = 17,
    Setting_LimitPullUpDisableMask = 18,
    Setting_ProbePullUpDisable = 19,
    Setting_SoftLimitsEnable = 20,
    Setting_HardLimitsEnable = 21,
    Setting_HomingEnable = 22,
    Setting_HomingDirMask = 23,
    Setting_HomingFeedRate = 24,
    Setting_HomingSeekRate = 25,
    Setting_HomingDebounceDelay = 26,
    Setting_HomingPulloff = 27,
    Setting_G73Retract = 28,
    Setting_PulseDelayMicroseconds = 29,
    Setting_RpmMax = 30,
    Setting_RpmMin = 31,
    Setting_Mode = 32,
    Setting_PWMFreq = 33,
    Setting_PWMOffValue = 34,
    Setting_PWMMinValue = 35,
    Setting_PWMMaxValue = 36,
    Setting_SteppersEnergize = 37,
    Setting_SpindlePPR = 38,
    Setting_EnableLegacyRTCommands = 39,
    Setting_JogSoftLimited = 40,
    Setting_ParkingEnable = 41,
    Setting_ParkingAxis = 42,

    Setting_HomingLocateCycles = 43,
    Setting_HomingCycle_1 = 44,
    Setting_HomingCycle_2 = 45,
    Setting_HomingCycle_3 = 46,
    Setting_HomingCycle_4 = 47,
    Setting_HomingCycle_5 = 48,
    Setting_HomingCycle_6 = 49,
// Optional driver implemented settings for jogging
    Setting_JogStepSpeed = 50,
    Setting_JogSlowSpeed = 51,
    Setting_JogFastSpeed = 52,
    Setting_JogStepDistance = 53,
    Setting_JogSlowDistance = 54,
    Setting_JogFastDistance = 55,
//
    Setting_ParkingPulloutIncrement = 56,
    Setting_ParkingPulloutRate = 57,
    Setting_ParkingTarget = 58,
    Setting_ParkingFastRate = 59,

    Setting_RestoreOverrides = 60,
    Setting_DoorOptions = 61,
    Setting_SleepEnable = 62,
    Setting_HoldActions = 63,
    Setting_ForceInitAlarm = 64,
    Setting_ProbingFeedOverride = 65,
// Optional driver implemented settings for piecewise linear spindle PWM algorithm
    Setting_LinearSpindlePiece1 = 66,
    Setting_LinearSpindlePiece2 = 67,
    Setting_LinearSpindlePiece3 = 68,
    Setting_LinearSpindlePiece4 = 69,
//

// Optional driver implemented settings for additional streams
    Setting_NetworkServices = 70,
    Setting_BlueToothDeviceName = 71,
    Setting_BlueToothServiceName = 72,
    Setting_WifiMode = 73,
    Setting_WiFi_STA_SSID = 74,
    Setting_WiFi_STA_Password = 75,
    Setting_WiFi_AP_SSID = 76,
    Setting_WiFi_AP_Password = 77,
    Setting_Wifi_AP_Country = 78,
    Setting_Wifi_AP_Channel = 79,
//

// Optional settings for closed loop spindle speed control
    Setting_SpindlePGain = 80,
    Setting_SpindleIGain = 81,
    Setting_SpindleDGain = 82,
    Setting_SpindleDeadband = 83,
    Setting_SpindleMaxError = 84,
    Setting_SpindleIMaxError = 85,
    Setting_SpindleDMaxError = 86,

// Optional settings for closed loop spindle synchronized motion
    Setting_PositionPGain = 90,
    Setting_PositionIGain = 91,
    Setting_PositionDGain = 92,
    Setting_PositionDeadband = 93,
    Setting_PositionMaxError = 94,
    Setting_PositionIMaxError = 95,
    Setting_PositionDMaxError = 96,
//

// Reserving settings in the range 100 - 299 for axis settings.
    Setting_AxisSettingsBase = 100,     // Reserved for core settings
    Setting_AxisSettingsMax = Setting_AxisSettingsBase + AXIS_SETTINGS_INCREMENT * 9 + N_AXIS,
    Setting_AxisSettingsBase2 = 200,    // Reserved for driver/plugin settings
    Setting_AxisSettingsMax2 = Setting_AxisSettingsBase2 + AXIS_SETTINGS_INCREMENT * 9 + N_AXIS,
//

// Optional driver implemented settings

    // Normally used for Ethernet
    Setting_Hostname = 300,
    Setting_IpMode = 301,
    Setting_IpAddress = 302,
    Setting_Gateway = 303,
    Setting_NetMask = 304,
    Setting_TelnetPort = 305,
    Setting_HttpPort = 306,
    Setting_WebSocketPort = 307,
    Setting_FtpPort = 308,

    // Normally used for WiFi Access Point
    Setting_Hostname2 = 310,
    Setting_IpMode2 = 311,
    Setting_IpAddress2 = 312,
    Setting_Gateway2 = 313,
    Setting_NetMask2 = 314,
    Setting_TelnetPort2 = 315,
    Setting_HttpPort2 = 316,
    Setting_WebSocketPort2 = 317,
    Setting_FtpPort2 = 318,

    // Normally used for WiFi Station
    Setting_Hostname3 = 320,
    Setting_IpMode3 = 321,
    Setting_IpAddress3 = 322,
    Setting_Gateway3 = 323,
    Setting_NetMask3 = 324,
    Setting_TelnetPort3 = 325,
    Setting_HttpPort3 = 326,
    Setting_WebSocketPort3 = 327,
    Setting_FtpPort3 = 328,

    Setting_AdminPassword = 330,
    Setting_UserPassword = 331,
    Setting_NTPServerURI = 332,
    Setting_NTPServerURI_2 = 333,
    Setting_NTPServerURI_3 = 334,
    Setting_Timezone = 335,
    Setting_DSTActive = 336,

    Setting_Wifi_AP_BSSID = 337,

    Setting_TrinamicDriver = 338,
    Setting_TrinamicHoming = 339,

    Setting_SpindleAtSpeedTolerance = 340,
    Setting_ToolChangeMode = 341,
    Setting_ToolChangeProbingDistance = 342,
    Setting_ToolChangeFeedRate = 343,
    Setting_ToolChangeSeekRate = 344,
    Setting_ToolChangePulloffRate = 345,
    Setting_ToolChangeRestorePosition = 346,

    Setting_DualAxisLengthFailPercent = 347,
    Setting_DualAxisLengthFailMin = 348,
    Setting_DualAxisLengthFailMax = 349,

    Setting_THC_Mode = 350,
    Setting_THC_Delay = 351,
    Setting_THC_Threshold = 352,
    Setting_THC_PGain = 353,
    Setting_THC_IGain = 354,
    Setting_THC_DGain = 355,
    Setting_THC_VADThreshold = 356,
    Setting_THC_VoidOverride = 357,
    Setting_Arc_FailTimeout = 358,
    Setting_Arc_RetryDelay = 359,
    Setting_Arc_MaxRetries = 360,
    Setting_Arc_VoltageScale = 361,
    Setting_Arc_VoltageOffset = 362,
    Setting_Arc_HeightPerVolt = 363,
    Setting_Arc_OkHighVoltage = 364,
    Setting_Arc_OkLowVoltage = 365,
    Setting_Arc_VoltagePort = 366,
    Setting_Arc_OkPort = 367,
    Setting_THC_CutterDownPort = 368,
    Setting_THC_CutterUpPort = 369,

    Settings_IoPort_InvertIn  = 370,
    Settings_IoPort_Pullup_Disable = 371,
    Settings_IoPort_InvertOut = 372,
    Settings_IoPort_OD_Enable = 373,
    Settings_ModBus_BaudRate = 374,
    Settings_ModBus_RXTimeout = 375,
    Settings_RotaryAxes = 376,
    Setting_BlueToothInitOK = 377,
    Setting_LaserCoolantOnDelay = 378,
    Setting_LaserCoolantOffDelay = 379,
    Setting_LaserCoolantMinTemp = 380,
    Setting_LaserCoolantMaxTemp = 381,
    Setting_LaserCoolantOffset = 382,
    Setting_LaserCoolantGain = 383,
    Setting_DisableG92Persistence = 384,
    Setting_BlueToothStateInput = 385,
    Setting_FanPort0 = 386,
    Setting_FanPort1 = 387,
    Setting_FanPort2 = 388,
    Setting_FanPort3 = 389,
    Setting_LaserCoolantTempPort = 390,
    Setting_LaserCoolantOkPort = 391,
    Setting_DoorSpindleOnDelay = 392,
    Setting_DoorCoolantOnDelay = 393,
    Setting_SpindleOnDelay = 394,
    Setting_SpindleType = 395,
    Setting_WebUiTimeout = 396,
    Setting_WebUiAutoReportInterval = 397,
    Setting_PlannerBlocks = 398,
    Setting_CANbus_BaudRate = 399,

    Setting_EncoderSettingsBase = 400, // NOTE: Reserving settings values >= 400 for encoder settings. Up to 449.
    Setting_EncoderSettingsMax = 449,

    // Reserved for user plugins - do NOT use for public plugins
    Setting_UserDefined_0 = 450,
    Setting_UserDefined_1 = 451,
    Setting_UserDefined_2 = 452,
    Setting_UserDefined_3 = 453,
    Setting_UserDefined_4 = 454,
    Setting_UserDefined_5 = 455,
    Setting_UserDefined_6 = 456,
    Setting_UserDefined_7 = 457,
    Setting_UserDefined_8 = 458,
    Setting_UserDefined_9 = 459,

    Setting_VFD_ModbusAddress = 460,
    Setting_VFD_RPM_Hz = 461,
    Setting_VFD_10 = 462,
    Setting_VFD_11 = 463,
    Setting_VFD_12 = 464,
    Setting_VFD_13 = 465,
    Setting_VFD_14 = 466,
    Setting_VFD_15 = 467,
    Setting_VFD_16 = 468,
    Setting_VFD_17 = 469,
    Setting_VFD_18 = 470,
    Setting_VFD_19 = 471,
    Setting_VFD_20 = 472,
    Setting_VFD_21 = 473,

    Setting_VFD_ModbusAddress0 = 476,
    Setting_VFD_ModbusAddress1 = 477,
    Setting_VFD_ModbusAddress2 = 478,
    Setting_VFD_ModbusAddress3 = 479,

    Setting_Fan0OffDelay = 480,
    Setting_AutoReportInterval = 481,
    Setting_TimeZoneOffset = 482,
    Setting_FanToSpindleLink = 483,
    Setting_UnlockAfterEStop = 484,
    Setting_EnableToolPersistence = 485,
    Setting_OffsetLock = 486,
    Setting_Spindle_OnPort = 487,
    Setting_Spindle_DirPort = 488,
    Setting_Spindle_PWMPort = 489,

    Setting_Macro0    = 490,
    Setting_MacroBase = Setting_Macro0,
    Setting_Macro1    = 491,
    Setting_Macro2    = 492,
    Setting_Macro3    = 493,
    Setting_Macro4    = 494,
    Setting_Macro5    = 495,
    Setting_Macro6    = 496,
    Setting_Macro7    = 497,
    Setting_Macro8    = 498,
    Setting_Macro9    = 499,

    Setting_MacroPort0    = 500,
    Setting_MacroPortBase = Setting_MacroPort0,
    Setting_MacroPort1    = 501,
    Setting_MacroPort2    = 502,
    Setting_MacroPort3    = 503,
    Setting_MacroPort4    = 504,
    Setting_MacroPort5    = 505,
    Setting_MacroPort6    = 506,
    Setting_MacroPort7    = 507,
    Setting_MacroPort8    = 508,
    Setting_MacroPort9    = 509,

    Setting_SpindleEnable0    = 510,
    Setting_SpindleEnableBase = Setting_SpindleEnable0,
    Setting_SpindleEnable1    = 511,
    Setting_SpindleEnable2    = 512,
    Setting_SpindleEnable3    = 513,
    Setting_SpindleEnable4    = 514,
    Setting_SpindleEnable5    = 515,
    Setting_SpindleEnable6    = 516,
    Setting_SpindleEnable7    = 517,
    Setting_EncoderSpindle    = 519,

    Setting_SpindleToolStart0    = 520,
    Setting_SpindleToolStartBase = Setting_SpindleToolStart0,
    Setting_SpindleToolStart1    = 521,
    Setting_SpindleToolStart2    = 522,
    Setting_SpindleToolStart3    = 523,
    Setting_SpindleToolStart4    = 524,
    Setting_SpindleToolStart5    = 525,
    Setting_SpindleToolStart6    = 526,
    Setting_SpindleToolStart7    = 527,

    Setting_MQTTBrokerIpAddress = 530,
    Setting_MQTTBrokerPort      = 531,
    Setting_MQTTBrokerUserName  = 532,
    Setting_MQTTBrokerPassword  = 533,

    Setting_NGCDebugOut = 534,
    Setting_NetworkMAC = 535,
    Setting_RGB_StripLengt0 = 536,
    Setting_RGB_StripLengt1 = 537,
    Setting_RotaryWrap = 538,
    Setting_SpindleOffDelay = 539,

    Setting_Panel_SpindleSpeed       = 540,  // NOTE: Reserving settings values 540 to 579 for panel settings.
    Setting_Panel_ModbusAddress      = 541,
    Setting_Panel_UpdateInterval     = 542,
    Setting_Panel_JogSpeed_x1        = 543,
    Setting_Panel_JogSpeed_x10       = 544,
    Setting_Panel_JogSpeed_x100      = 545,
    Setting_Panel_JogSpeed_Keypad    = 546,
    Setting_Panel_JogDistance_x1     = 547,
    Setting_Panel_JogDistance_x10    = 548,
    Setting_Panel_JogDistance_x100   = 549,
    Setting_Panel_JogDistance_Keypad = 550,
    Setting_Panel_JogAccelRamp       = 551,
    Setting_Panel_Encoder0_Mode      = 552,
    Setting_Panel_Encoder0_Cpd       = 553,
    Setting_Panel_Encoder1_Mode      = 554,
    Setting_Panel_Encoder1_Cpd       = 555,
    Setting_Panel_Encoder2_Mode      = 556,
    Setting_Panel_Encoder2_Cpd       = 557,
    Setting_Panel_Encoder3_Mode      = 558,
    Setting_Panel_Encoder3_Cpd       = 559,
    Setting_Panel_SettingsMax        = 579,

    Setting_ButtonAction0    = 590,
    Setting_ButtonActionBase = Setting_ButtonAction0,
    Setting_ButtonAction1    = 591,
    Setting_ButtonAction2    = 592,
    Setting_ButtonAction3    = 593,
    Setting_ButtonAction4    = 594,
    Setting_ButtonAction5    = 595,
    Setting_ButtonAction6    = 596,
    Setting_ButtonAction7    = 597,
    Setting_ButtonAction8    = 598,
    Setting_ButtonAction9    = 599,

    Setting_ModbusTCPBase       = 600,    // Reserving settings values 600 to 639 for ModBus TCP (8 sets)
    Setting_ModbusIpAddressBase = Setting_ModbusTCPBase + Setting_ModbusIpAddress,
    Setting_ModbusPortBase      = Setting_ModbusTCPBase + Setting_ModbusPort,
    Setting_ModbusIdBase        = Setting_ModbusTCPBase + Setting_ModbusId,
    Setting_ModbusTCPMax        = 639,

    Setting_Kinematics0         = 640,
    Setting_Kinematics1         = 641,
    Setting_Kinematics2         = 642,
    Setting_Kinematics3         = 643,
    Setting_Kinematics4         = 644,
    Setting_Kinematics5         = 645,
    Setting_Kinematics6         = 646,
    Setting_Kinematics7         = 647,
    Setting_Kinematics8         = 648,
    Setting_Kinematics9         = 649,

    Setting_FSOptions = 650,

    Setting_Stepper1  = 651,
    Setting_Stepper2  = 652,
    Setting_Stepper3  = 653,
    Setting_Stepper4  = 654,
    Setting_Stepper5  = 655,
    Setting_Stepper6  = 656,
    Setting_Stepper7  = 657,
    Setting_Stepper8  = 658,
    Setting_Stepper9  = 659,
    Setting_Stepper10 = 660,
    Setting_Stepper11 = 661,
    Setting_Stepper12 = 662,
    Setting_Stepper13 = 663,
    Setting_Stepper14 = 664,
    Setting_Stepper15 = 665,
    Setting_Stepper16 = 666,
    Setting_Stepper17 = 667,
    Setting_Stepper18 = 668,
    Setting_Stepper19 = 669,
    Setting_Stepper20 = 670,

    Setting_HomePinsInvertMask = 671,
    Setting_Reserved672 = 672,
    Setting_CoolantOnDelay = 673,

    Setting_SpindleInvertMask1 = 716,

    Setting_RpmMax1 = 730,
    Setting_RpmMin1 = 731,
    Setting_Mode1 = 732,
    Setting_PWMFreq1 = 733,
    Setting_PWMOffValue1 = 734,
    Setting_PWMMinValue1 = 735,
    Setting_PWMMaxValue1 = 736,

// Optional driver implemented settings for piecewise linear spindle PWM algorithm
    Setting_LinearSpindle1Piece1 = 737,
    Setting_LinearSpindle1Piece2 = 738,
    Setting_LinearSpindle1Piece3 = 739,
    Setting_LinearSpindle1Piece4 = 740,

    Setting_Action0    = 750,
    Setting_ActionBase = Setting_Action0,
    Setting_Action1    = 751,
    Setting_Action2    = 752,
    Setting_Action3    = 753,
    Setting_Action4    = 754,
    Setting_Action5    = 755,
    Setting_Action6    = 756,
    Setting_Action7    = 757,
    Setting_Action8    = 758,
    Setting_Action9    = 759,

    Setting_ActionPort0    = 760,
    Setting_ActionPortBase = Setting_ActionPort0,
    Setting_ActionPort1    = 761,
    Setting_ActionPort2    = 762,
    Setting_ActionPort3    = 763,
    Setting_ActionPort4    = 764,
    Setting_ActionPort5    = 765,
    Setting_ActionPort6    = 766,
    Setting_ActionPort7    = 767,
    Setting_ActionPort8    = 768,
    Setting_ActionPort9    = 769,

    Setting_SpindleOffsetX = 770,
    Setting_SpindleOffsetY = 771,
//
// 772-779 - reserved for spindle offset settings
//

// Reserving settings in the range 800 - 899 for axis settings.
    Setting_AxisSettingsBase1 = 800,    // Reserved for driver/plugin settings
    Setting_AxisSettingsMax1 = Setting_AxisSettingsBase1 + AXIS_SETTINGS_INCREMENT * 9 + N_AXIS,
//

//
// 900-999 - reserved for automatic tool changers (ATC)
//

// ---
    Setting_SettingsMax,
    Setting_SettingsAll = Setting_SettingsMax,
// ---

    // Calculated base values for core stepper settings
    Setting_AxisStepsPerMM       = Setting_AxisSettingsBase,
    Setting_AxisMaxRate          = Setting_AxisSettingsBase + AXIS_SETTINGS_INCREMENT,
    Setting_AxisAcceleration     = Setting_AxisSettingsBase + 2 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisMaxTravel        = Setting_AxisSettingsBase + 3 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisStepperCurrent   = Setting_AxisSettingsBase + 4 * AXIS_SETTINGS_INCREMENT, // Not used by the core
    Setting_AxisMicroSteps       = Setting_AxisSettingsBase + 5 * AXIS_SETTINGS_INCREMENT, // Not used by the core
    Setting_AxisBacklash         = Setting_AxisSettingsBase + 6 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisAutoSquareOffset = Setting_AxisSettingsBase + 7 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisHomingFeedRate   = Setting_AxisSettingsBase + 8 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisHomingSeekRate   = Setting_AxisSettingsBase + 9 * AXIS_SETTINGS_INCREMENT,

    Setting_AxisJerk             = Setting_AxisSettingsBase1,

    // Calculated base values for driver/plugin stepper settings
    Setting_AxisExtended0        = Setting_AxisSettingsBase2,
    Setting_AxisExtended1        = Setting_AxisSettingsBase2 + AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended2        = Setting_AxisSettingsBase2 + 2 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended3        = Setting_AxisSettingsBase2 + 3 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended4        = Setting_AxisSettingsBase2 + 4 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended5        = Setting_AxisSettingsBase2 + 5 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended6        = Setting_AxisSettingsBase2 + 6 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended7        = Setting_AxisSettingsBase2 + 7 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended8        = Setting_AxisSettingsBase2 + 8 * AXIS_SETTINGS_INCREMENT,
    Setting_AxisExtended9        = Setting_AxisSettingsBase2 + 9 * AXIS_SETTINGS_INCREMENT,

    // Calculated base values for encoder settings
    Setting_EncoderModeBase           = Setting_EncoderSettingsBase + Setting_EncoderMode,
    Setting_EncoderCPRBase            = Setting_EncoderSettingsBase + Setting_EncoderCPR,
    Setting_EncoderCPDBase            = Setting_EncoderSettingsBase + Setting_EncoderCPD,
    Setting_EncoderDblClickWindowBase = Setting_EncoderSettingsBase + Setting_EncoderDblClickWindow
} setting_id_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t defaults          :1,
                parameters        :1,
                startup_lines     :1,
                build_info        :1,
                driver_parameters :1,
                unassigned        :3;

    };
} settings_restore_t;

extern const settings_restore_t settings_all;

typedef char stored_line_t[MAX_STORED_LINE_LENGTH];

typedef union {
    uint32_t value;
    struct {
        uint32_t report_inches                   :1,
                 restore_overrides               :1,
                 dst_active                      :1, // Daylight savings time
                 sleep_enable                    :1,
                 disable_laser_during_hold       :1,
                 force_initialization_alarm      :1,
                 legacy_rt_commands              :1,
                 restore_after_feed_hold         :1,
                 ngc_debug_out                   :1,
                 g92_is_volatile                 :1,
                 compatibility_level             :4,
                 no_restore_position_after_M6    :1,
                 no_unlock_after_estop           :1,
                 settings_downgrade              :1,
         		 unassigned                      :15;
    };
} settingflags_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t invert_probe_pin          :1,
                 disable_probe_pullup      :1,
                 invert_connected_pin      :1,
                 disable_connected_pullup  :1,
                 allow_feed_override       :1,
                 enable_protection         :1,
                 invert_toolsetter_input   :1,
                 disable_toolsetter_pullup :1,
                 unassigned                :8;
    };
} probeflags_t;

typedef union {
    uint16_t mask;
    struct {
        uint16_t machine_position   :1,
                 buffer_state       :1,
                 line_numbers       :1,
                 feed_speed         :1,
                 pin_state          :1,
                 work_coord_offset  :1,
                 overrides          :1,
                 probe_coordinates  :1,
                 sync_on_wco_change :1,
                 parser_state       :1,
                 alarm_substate     :1,
                 run_substate       :1,
                 when_homing        :1,
                 unassigned         :3;
    };
} reportmask_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t ignore_when_idle :1,
                keep_coolant_on  :1,
                unassigned       :6;
    };
} safety_door_setting_flags_t;

typedef struct {
    safety_door_setting_flags_t flags; // TODO: move to last element in next revision
    float spindle_on_delay; // TODO: change to uint16_t in next revision
    float coolant_on_delay; // TODO: change to uint16_t in next revision
} safety_door_settings_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled                 :1,
                deactivate_upon_init    :1,
                enable_override_control :1,
                unassigned              :5;
    };
} parking_setting_flags_t;

typedef struct {
    parking_setting_flags_t flags;
    uint8_t axis;               // Define which axis that performs the parking motion
    float target;               // Parking axis target. In mm, as machine coordinate [-max_travel,0].
    float rate;                 // Parking fast rate after pull-out in mm/min.
    float pullout_rate;         // Pull-out/plunge slow feed rate in mm/min.
    float pullout_increment;    // Spindle pull-out and plunge distance in mm. Incremental distance.
} parking_settings_t;

typedef struct {
    pid_values_t pid;
} position_pid_t; // Used for synchronized motion

typedef union {
    uint16_t value;
    struct {
        uint16_t enabled              :1,
                 single_axis_commands :1,
                 init_lock            :1,
                 force_set_origin     :1,
                 two_switches         :1, // -> limits.flags.two_switches, never set
                 manual               :1,
                 override_locks       :1,
                 keep_on_reset        :1,
                 use_limit_switches   :1,
                 per_axis_feedrates   :1,
                 unused               :6;
    };
} homing_settings_flags_t;

typedef struct {
    float fail_length_percent; // DEFAULT_DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT
    float fail_distance_max;   // DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX
    float fail_distance_min;   // DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN
} homing_dual_axis_t;

typedef struct {
    float pulloff;
    homing_dual_axis_t dual_axis;
    uint16_t debounce_delay;
    homing_settings_flags_t flags;
    axes_signals_t dir_mask;
    uint8_t locate_cycles;
    axes_signals_t cycle[N_AXIS];
} homing_settings_t;

typedef struct {
    axes_signals_t step_invert;
    axes_signals_t dir_invert;
    axes_signals_t ganged_dir_invert; // applied after inversion for the master motor
    axes_signals_t enable_invert;
    axes_signals_t energize;
#if N_AXIS > 3
    axes_signals_t is_rotary;       // rotary axes distances are not scaled in imperial mode
    axes_signals_t rotary_wrap;     // rotary axes that allows G28 wrap for faster move to home position
#endif
    float pulse_microseconds;
    float pulse_delay_microseconds;
    uint16_t idle_lock_time; // If value = 255, steppers do not disable.
} stepper_settings_t;

typedef struct {
    float steps_per_mm;
    float max_rate;
    float acceleration;
    float max_travel;
    float dual_axis_offset;
    float homing_seek_rate;
    float homing_feed_rate;
    float jerk;
#if ENABLE_BACKLASH_COMPENSATION
    float backlash;
#endif
} axis_settings_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t hard_enabled         :1,
                check_at_init        :1,
                jog_soft_limited     :1,
                two_switches         :1,
                hard_disabled_rotary :1,
                unassigned           :3;
    };
} limit_settings_flags_t;

typedef struct {
    limit_settings_flags_t flags;
    axes_signals_t invert;
    axes_signals_t disable_pullup;
    axes_signals_t soft_enabled;
} limit_settings_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t sd_mount_on_boot  :1,
                lfs_hidden        :1,
                unused            :6;
    };
} fs_options_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t g59_1  :1,
                g59_2  :1,
                g59_3  :1,
                unused :5;
    };
} offset_lock_t;

typedef union {
    uint32_t value;
    uint32_t mask;
    struct {
        uint32_t bit0  :1,
                 bit1  :1,
                 bit2  :1,
                 bit3  :1,
                 bit4  :1,
                 bit5  :1,
                 bit6  :1,
                 bit7  :1,
                 bit8  :1,
                 bit9  :1,
                 bit10 :1,
                 bit11 :1,
                 bit12 :1,
                 bit13 :1,
                 bit14 :1,
                 bit15 :1;
    };
} ioport_bus_t;

typedef struct {
    ioport_bus_t invert_in;
    ioport_bus_t pullup_disable_in;
    ioport_bus_t invert_out;
    ioport_bus_t od_enable_out;
} ioport_signals_t;

typedef enum {
    ToolChange_Disabled = 0,
    ToolChange_Manual,
    ToolChange_Manual_G59_3,
    ToolChange_SemiAutomatic,
    ToolChange_Ignore
} toolchange_mode_t;

typedef struct {
    uint8_t length0;
    uint8_t length1;
} rgb_strip_settings_t;

typedef struct {
    toolchange_mode_t mode;
    float feed_rate;
    float seek_rate;
    float pulloff_rate;
    float probing_distance;
} tool_change_settings_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t id    :8;  // = SETTINGS_VERSION, incremented on structure changes.
        uint32_t build :24; // Build date, format YYMMDD.
    };
} settings_version_t;

// Global persistent settings (Stored from byte NVS_ADDR_GLOBAL onwards)
typedef struct {
    // Settings struct version
    settings_version_t version;
    float junction_deviation;
    float arc_tolerance;
    float g73_retract;
    float timezone;
    uint16_t report_interval;
    uint16_t planner_buffer_blocks;
    machine_mode_t mode;
    tool_change_settings_t tool_change;
    axis_settings_t axis[N_AXIS];
    control_signals_t control_invert;
    control_signals_t control_disable_pullup;
    axes_signals_t home_invert;
    coolant_settings_t coolant;
    uint8_t modbus_baud;
    uint8_t canbus_baud;
    spindle_settings_t spindle;
    spindle_pwm_settings_t pwm_spindle;
    stepper_settings_t steppers;
    reportmask_t status_report; // Mask to indicate desired report data.
    settingflags_t flags;       // Contains default boolean settings
    probeflags_t probe;
    rgb_strip_settings_t rgb_strip;
    offset_lock_t offset_lock;
    fs_options_t fs_options;
    limit_settings_t limits;
    parking_settings_t parking;
    safety_door_settings_t safety_door;
    position_pid_t position;    // Used for synchronized motion
    ioport_signals_t ioport;
    homing_settings_t homing;
    char reserved[24];          // Reserved For future expansion
} settings_t;

typedef enum {
    Group_Root = 0,             //!< 0
    Group_General,              //!< 1
    Group_ControlSignals,       //!< 2
    Group_Limits,               //!< 3
    Group_Limits_DualAxis,      //!< 4
    Group_Coolant,              //!< 5
    Group_Spindle,              //!< 6
    Group_Spindle_Sync,         //!< 7
    Group_Spindle_ClosedLoop,   //!< 8
    Group_Toolchange,           //!< 9
    Group_Plasma,               //!< 10
    Group_Homing,               //!< 11
    Group_Probing,              //!< 12
    Group_SafetyDoor,           //!< 13
    Group_Jogging,              //!< 14
    Group_Networking,           //!< 15
    Group_Networking_Wifi,      //!< 16
    Group_Bluetooth,            //!< 17
    Group_AuxPorts,             //!< 18
    Group_ModBus,               //!< 19
    Group_ModBusUnit0,          //!< 20
    Group_ModBusUnit1,          //!< 21
    Group_ModBusUnit2,          //!< 22
    Group_ModBusUnit3,          //!< 23
    Group_ModBusUnit4,          //!< 24
    Group_ModBusUnit5,          //!< 25
    Group_ModBusUnit6,          //!< 26
    Group_ModBusUnit7,          //!< 27
    Group_Encoders,             //!< 28
    Group_Encoder0,             //!< 29
    Group_Encoder1,             //!< 30
    Group_Encoder2,             //!< 31
    Group_Encoder3,             //!< 32
    Group_Encoder4,             //!< 33
    Group_UserSettings,         //!< 34
    Group_Stepper,              //!< 35
    Group_MotorDriver,          //!< 36
    Group_VFD,                  //!< 37
    Group_CANbus,               //!< 38
    Group_Embroidery,           //!< 39
    Group_Panel,                //!< 40
    Group_Kinematics,           //!< 41
    Group_Axis,                 //!< 42
// NOTE: axis groups MUST be sequential AND last
    Group_Axis0,                //!< 43
    Group_XAxis = Group_Axis0,  //!< 44
    Group_YAxis,                //!< 45
    Group_ZAxis,                //!< 46
#ifdef A_AXIS
    Group_AAxis,                //!< 47
#endif
#ifdef B_AXIS
    Group_BAxis,                //!< 48
#endif
#ifdef C_AXIS
    Group_CAxis,                //!< 49
#endif
#ifdef U_AXIS
    Group_UAxis,                //!< 50
#endif
#ifdef V_AXIS
    Group_VAxis,                //!< 51
#endif
    Group_Unknown = 99,         //!< 99
    Group_All = Group_Root      //!< 0
} setting_group_t;

typedef enum {
    Format_Bool = 0,
    Format_Bitfield,
    Format_XBitfield,
    Format_RadioButtons,
    Format_AxisMask,
    Format_Integer, // 32 bit
    Format_Decimal,
    Format_String,
    Format_Password,
    Format_IPv4,
    // For internal use only
    Format_Int8,
    Format_Int16,
} setting_datatype_t;

typedef struct setting_group_detail {
    setting_group_t parent;
    setting_group_t id;
    const char *name;
    bool (*is_available)(const struct setting_group_detail *group);
} setting_group_detail_t;

typedef enum {
    Setting_NonCore = 0,
    Setting_NonCoreFn,
    Setting_IsExtended,
    Setting_IsExtendedFn,
    Setting_IsLegacy,
    Setting_IsLegacyFn,
    Setting_IsExpanded,
    Setting_IsExpandedFn
} setting_type_t;

typedef union {
    uint32_t ivalue;
    float fvalue;
} setting_limit_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t reboot_required :1,
                allow_null      :1,
                subgroups       :1,
                increment       :4,
                hidden          :1; //!< Hide from reporting, allow setting
    };
} setting_detail_flags_t;

typedef struct setting_detail {
    setting_id_t id;
    setting_group_t group;
    const char *name;
    const char *unit;
    setting_datatype_t datatype;
    const char *format;
    const char *min_value;
    const char *max_value;
    setting_type_t type;
    void *value;
    void *get_value;
    bool (*is_available)(const struct setting_detail *setting);
    setting_detail_flags_t flags;
} setting_detail_t;

typedef struct {
    setting_id_t id;
    const char *description;
} setting_descr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t spindle    :1,
                unassigned :7;
    };
} settings_changed_flags_t;

typedef status_code_t (*setting_set_int_ptr)(setting_id_t id, uint_fast16_t value);
typedef status_code_t (*setting_set_float_ptr)(setting_id_t id, float value);
typedef status_code_t (*setting_set_string_ptr)(setting_id_t id, char *value);
typedef uint32_t (*setting_get_int_ptr)(setting_id_t id);
typedef float (*setting_get_float_ptr)(setting_id_t id);
typedef char *(*setting_get_string_ptr)(setting_id_t id);
typedef bool (*setting_output_ptr)(const setting_detail_t *setting, uint_fast16_t offset, void *data);

typedef void (*setting_changed_ptr)(setting_id_t id);

/*! \brief Pointer to callback function to be called when settings are loaded or changed.
\param settings pointer to \a settings_t struct containing the settings.
\param changed a \a settings_changed_flags_t union containing the changed setting groups.
*/
typedef void (*settings_changed_ptr)(settings_t *settings, settings_changed_flags_t changed);

typedef void (*driver_settings_load_ptr)(void);
typedef void (*driver_settings_save_ptr)(void);
typedef void (*driver_settings_restore_ptr)(void);
typedef bool (*driver_settings_iterator_ptr)(const setting_detail_t *setting, setting_output_ptr callback, void *data);

typedef struct setting_details {
    const uint8_t n_groups;
    const setting_group_detail_t *groups;
    const uint16_t n_settings;
    const setting_detail_t *settings;
#ifndef NO_SETTINGS_DESCRIPTIONS
    const uint16_t n_descriptions;
    const setting_descr_t *descriptions;
#endif
//    struct setting_details *(*on_get_settings)(void);
    struct setting_details *next;
    settings_changed_ptr on_changed;
    driver_settings_save_ptr save;
    driver_settings_load_ptr load;
    driver_settings_restore_ptr restore;
    driver_settings_iterator_ptr iterator;
} setting_details_t;

// NOTE: this must match the signature of on_get_settings in the setting_details_t structure above!
typedef setting_details_t *(*on_get_settings_ptr)(void);

extern settings_t settings;

// Clear settings chain (unlinks plugin/driver settings from core settings)
void settings_clear (void);

// Initialize the configuration subsystem (load settings from persistent storage)
void settings_init();

// Write Grbl global settings and version number to persistent storage
void settings_write_global(void);

// Helper function to clear and restore persistent storage defaults
void settings_restore(settings_restore_t restore_flags);

// A helper method to set new settings from command line
status_code_t settings_store_setting(setting_id_t setting, char *svalue);

// Writes the protocol line variable as a startup line in persistent storage
void settings_write_startup_line(uint8_t idx, char *line);

// Reads an persistent storage startup line to the protocol line variable
bool settings_read_startup_line(uint8_t idx, char *line);

// Writes build info user-defined string
void settings_write_build_info(char *line);

// Reads build info user-defined string
bool settings_read_build_info(char *line);

// Writes selected coordinate data to persistent storage
void settings_write_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Reads selected coordinate data from persistent storage
bool settings_read_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Temporarily override acceleration, if 0 restore to configured setting value
bool settings_override_acceleration (uint8_t axis, float acceleration);

void settings_register (setting_details_t *details);
setting_details_t *settings_get_details (void);
bool settings_is_group_available (setting_group_t group);
bool settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data);
const setting_detail_t *setting_get_details (setting_id_t id, setting_details_t **set);
const char *setting_get_description (setting_id_t id);
setting_datatype_t setting_datatype_to_external (setting_datatype_t datatype);
setting_group_t settings_normalize_group (setting_group_t group);
const setting_group_detail_t *setting_get_group_details (setting_group_t id);
char *setting_get_value (const setting_detail_t *setting, uint_fast16_t offset);
uint32_t setting_get_int_value (const setting_detail_t *setting, uint_fast16_t offset);
float setting_get_float_value (const setting_detail_t *setting, uint_fast16_t offset);
setting_id_t settings_get_axis_base (setting_id_t id, uint_fast8_t *idx);
bool setting_is_list (const setting_detail_t *setting);
bool setting_is_integer (const setting_detail_t *setting);
void setting_remove_elements (setting_id_t id, uint32_t mask);
bool settings_add_spindle_type (const char *type);
limit_signals_t settings_get_homing_source (void);

#endif
