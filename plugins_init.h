/*
  plugins_init.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Calls the init function of enabled plugins, may be included at the end of the drivers driver_init() implementation.

  These are NOT referenced in the core grbl code

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#if TRINAMIC_ENABLE
    extern bool trinamic_init (void);
    trinamic_init();
#endif

#if PLASMA_ENABLE
    extern void plasma_init (void);
    plasma_init();
#endif

#if MODBUS_ENABLE && (MODBUS_ENABLE & 0x01)
    extern void modbus_rtu_init (void);
    modbus_rtu_init();
#endif

#if CANBUS_ENABLE
    extern void canbus_init (void);
    canbus_init();
#endif

#if SPINDLE_ENABLE & (1<<SPINDLE_PWM0_CLONE)
    extern void cloned_spindle_init (void);
    cloned_spindle_init();
#endif

#if SPINDLE_ENABLE & (1<<SPINDLE_STEPPER)
    extern void stepper_spindle_init (void);
    stepper_spindle_init();
#endif

#if SPINDLE_ENABLE & ((1<<SPINDLE_ONOFF1)|(1<<SPINDLE_ONOFF1_DIR))
    extern void onoff_spindle_init (void);
    onoff_spindle_init();
#endif

#if SPINDLE_ENABLE & (1<<SPINDLE_PWM2)
    extern void pwm_spindle_init (void);
    pwm_spindle_init();
#endif

#if VFD_ENABLE
    extern void vfd_init (void);
    vfd_init();
#endif

#if BLUETOOTH_ENABLE > 1
    extern void bluetooth_init (void);
    bluetooth_init();
#endif

#if ESP_AT_ENABLE
    extern void esp_at_init (void);
    esp_at_init();
#endif

#if KEYPAD_ENABLE
    extern bool keypad_init (void);
    keypad_init();
#endif

#if MACROS_ENABLE
    extern bool macros_init (void);
    macros_init();
#endif

#if LASER_COOLANT_ENABLE
    extern void laser_coolant_init (void);
    laser_coolant_init();
#endif

#if FANS_ENABLE
    extern void fans_init (void);
    fans_init();
#endif

#if OPENPNP_ENABLE
    extern void openpnp_init (void);
    openpnp_init();
#endif

#if LB_CLUSTERS_ENABLE
    extern void lb_clusters_init (void);
    lb_clusters_init();
#endif

#if PROBE_PROTECT_ENABLE
    extern void probe_protect_init (void);
    probe_protect_init();
#endif

#if WEBUI_ENABLE
    extern void webui_init (void);
    webui_init();
#endif

#if EMBROIDERY_ENABLE
    extern void embroidery_init (void);
    embroidery_init();
#endif

#if RGB_LED_ENABLE
    extern void rgb_led_init (void);
    rgb_led_init();
#endif

#if FEED_OVERRIDE_ENABLE
    extern void feed_override_init (void);
    feed_override_init();
#endif

#if HOMING_PULLOFF_ENABLE
    extern void homing_pulloff_init (void);
    homing_pulloff_init();
#endif

    extern void my_plugin_init (void);
    my_plugin_init();

#if N_SPINDLE > 1
    extern void spindle_select_init(void);
    spindle_select_init();
  #if SPINDLE_OFFSET == 1
    extern void spindle_offset_init (void);
    spindle_offset_init();
  #endif
#endif

// Third party plugin definitions.
// The code for these has to be downloaded from the source and placed in the same folder as driver.c
// Note: Third party plugins may have more than one implementation, there is no "owner" of plugins listed here.
//       It is also guaranteed that there will be no implementation in the grblHAL repository of these.

#if PWM_SERVO_ENABLE
    extern void pwm_servo_init (void);
    pwm_servo_init();
#endif

#if BLTOUCH_ENABLE
    extern void bltouch_init (void);
    bltouch_init();
#endif

#if ATC_ENABLE
    extern void atc_init (void);
    atc_init();
#endif

#if PROBE_RELAY_ENABLE
    extern void probe_relay_init (void);
    probe_relay_init();
#endif

#if DISPLAY_ENABLE
    void display_init (void);
    display_init();
#endif

#if STATUS_LIGHT_ENABLE
    extern void status_light_init (void);
    status_light_init();
#endif

#if PANEL_ENABLE
    extern void panel_init (void);
    panel_init();
#endif

#if EVENTOUT_ENABLE
    extern void event_out_init (void);
    event_out_init();
#endif

// End third party plugin definitions.

#if ODOMETER_ENABLE
    extern void odometer_init (void);
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

/*EOF*/
