## grblHAL changelog

<a name="20250102">20250102

Core:

* Fix for coolant issue when resetting from feed hold state. Ref. STMF32F4xx issue [#205](https://github.com/grblHAL/STM32F4xx/issues/205).

Drivers:

* STM32F4xx, STM32F7xx: some minor fixes.

Plugins:

* Spindle: fix for alarm 14 beeing raised on reset. Ref. STMF32F4xx issue [#205](https://github.com/grblHAL/STM32F4xx/issues/205).  
Added retry handling to Modbus RTU driver, updated VFD spindle drivers accordingly. 

---

<a name="20250101">20250101

Drivers: 

* ESP32: fixed regression causing compiler failure if Bluetooth is enabled.

* iMXRT1062: fixed regression causing compiler failure if laser PPI plugin is enabled.

* STM32F1xx, STM32F4xx: updates for Web Builder.

Plugins:

* Laser, PPI: added disable of PPI mode on program end \(`M2`, `M30`\) and soft reset.

---

<a name="20241226">Build 20241230

Core:

* Fix for laser incorrectly enabled in laser mode when `M3S<n>` commanded in `G0` and `G80` modal states. Ref. issue [#644](https://github.com/grblHAL/core/issues/644).

* Added support for 3rd order acceleration \(jerk\) and G187 gcode. Ref. pull request [#593](https://github.com/grblHAL/core/pull/593).

Drivers:

* STM32F4xx: updated ST framework to latest version, added support for ethernet via DP83848 PHY.

* STM32F4xx, STM32F7xx: improved SD card mount/dismount handling and added support for card detect signal.  
Now flags RTC as set if date >= grblHAL build date.

* LPC176x: added function required for Modbus support. Renamed bootloader build option to avoid confusion.

Plugins:

* SD card: improved mount/dismount handling.

---

<a name="20241226">20241226

Drivers:

* STM32F4xx: SuperLongBoard - fix for not enabling steppers after clearing E-stop alarm. Ref. issue [#203](https://github.com/grblHAL/STM32F4xx/issues/203).

Plugins:

* Networking: fix for not bringing up the network stack on link aquired when static IP configured, affects WizNet modules.

* Motors: fix for "typo" blocking StealthChop mode for TMC2209 drivers when it should be for TMC2660. Ref. issue [#17](https://github.com/grblHAL/Plugins_motor/issues/17).

--

<a name="20241222">20241222

Drivers:

* iMXRT1062: added fix for not bringing up the network stack on link aquired when static IP configured.  
Fixes for boards not booting when pin controlled MPG mode option selected.

---

<a name="20241222">Build 20241222

Core:

* Improved handling of real-time clock \(RTC\).

Drivers:

* ESP32: added support for 4-lane SDIO SD card interface and SD card detect input. Updated RTC handling to match core.

* iMXRT1062: fix for littlefs issue. Ref. [this comment](https://github.com/grblHAL/core/discussions/203#discussioncomment-11646077).

* RP2040, STM32F4xx, STM3F27xx: updated RTC handling to match core.

Plugins:

* SD card, Networking: added missing include.

---

<a name="20241219">Build 20241219

Core:

* Added directory \(mounts\) listing capabilities to the default root file system when no real root file system \(SD card\) is mounted.

Boards:

* ESP32, iMXRT1062, RP2040: added option to mount littlefs as root filesystem when SD card is not enabled.  
Ref. RP2040 issue [#103](https://github.com/grblHAL/RP2040/issues/103).

* STM32F3xx: added serial port to `$pins` report.

Plugins:

* WebUI, Networking and SD card: updated to handle littlefs mounted as root filesystem.

---

## grblHAL changelog

<a name="20241217">Build 20241217

Core:

* Added preprocessor support for moving coolant outputs to auxiliary pool. Some minor bug fixes and code cleanup.  
Those who have custom board maps must update pin assignments accordingly when updating to this or later versions.

Drivers:

* Most: moved coolant outputs to auxiliary outputs pool for many boards.  
Web Builder functionality for assigning those outputs as coolant or as auxiliary (controlled by M62-M65) will be forthcoming.

* ESP32: added driver support for second PWM spindle, only configurable for the MKS DLC32 v2 board for now.

Plugins:

* Spindle: removed obsoleted code, fix for second PWM spindle w/o direction output.

<a name="20241214">Build 20241214

Core:

* Added suppression of door open signal when in manual or semi-automatic tool change mode.

Drivers:

* ESP32: added board map for BTT Rodent. Untested!

* STM32F7xx: added `$DFU` system command for entering DFU programming mode.

Plugins:

* Misc, ESP-AT: added compile time symbol for selecting stream to use.

---

<a name="20241212">20241212

Drivers:

* ESP32: added generic map for ESP32-S3, refactored UART and lowlevel Trinamic driver code. Removed superfluous definitions in board maps.

* RP2040: fix for Ethernet config errors when no WizNet module is installed.

Plugins:

* Trinamic: disabled current settings pot for TMC2209 drivers. __NOTE:__ this may result in a larger current than expected flowing, if motors runs hot readjust!

---

<a name="20241210">Build 20241210

Core:

* Increased preprocessor support for up to 16 auxiliary input pins, used by STM32* drivers.

Drivers:

* STM32F4xx: fix for Superlongboard \(SLB\) not enabling stepper drivers after E-Stop.

Plugins:

* Motors: added API call for reinitializing stepper drivers.

---

<a name="20241208">Build 20241208

Core:

* Revised core setting structures, changed from 8-bit to 16-bit CRC checksums for improved detection of corruption/version mismatches.  
__NOTE:__ Backup and restore settings over an update since _all_ settings will be reset to default. Any odometer data will also be lost.

* Added option to homing enable setting (`$22`) for per axis homing feedrates.  
When this option is selected setting `$24` and $`25` will be disabled and new axis settings made available;
`$18<n>` replaces `$24` and `$19<n>` replaces `$25`. `<n>` is the axis number; `0` for X, `1` for Y, ...  
__NOTE:__ if axes are set up for simultaneous homing and they do not have the same feedrates they will be homed separately.  
__NOTE:__ `$18<n>` and `$19<n>` were previousely implemented by the Trinamic motor plugin, the implementation is now in the core.  
__NOTE:__ core settings will now overflow the legacy 1024 byte boundary when > 5 axes are configured, in the previous version when > 6 axes were configured.

Drivers:

* All: updated for the revised settings structures.

* iMXRT1062, RP2040 and SAM3X8E: improved step injection option used by the Plasma plugin and stepper spindle option.

* RP2040: tuning for the new RP2350 MCU, fixes step timings.

Plugins:

* Some updated for the revised settings structures.

---

<a name="20241205">20241205

Drivers:

* RP2040: initial support for RP2350 \(Pico 2\) added to Web Builder.

Plugins:

* Trinamic: fixed min/max current calculations for TMC5160 driver. Ref. [issue #16](https://github.com/grblHAL/Plugins_motor/issues/16);

---

<a name="20241204">20241204

Core:

* Changed deprecated calls to `isinff()` and `isnanf()` to new \(C99\) versions.

Drivers:

* RP2040: CMakeLists.txt fixes for RP2350 builds.

* STM32F4xx: disabled broken support for sensorless homing for LongBoard32 \(SLB\). Support for this board is still WIP.

Plugins:

* Motors , spindle and misc: changed deprecated calls to `isinff()` and `isnanf()` to new \(C99\) versions.

---

<a name="20241128">Build 20241128

Core:

* Added (or rather repurposed) field for build date to settings structure in preparation for coming changes.

* Added code guards in order to free some memory for STM32F103 variants with 128K flash. Ref. [STM32F1xx driver issue #59](https://github.com/grblHAL/STM32F1xx/issues/59).

* Changed Stop (`0x19`) real-time command behaviour, active tool offset and coordinate system will now be kept. Ref. [issue #610](https://github.com/grblHAL/core/issues/610).  

Drivers:

* All: updated for core change (settings structure).

* SAM3X8E: fixed missing probe input for Protoneer v3 board. Ref. [issue #31](https://github.com/grblHAL/SAM3X8E/issues/31)

* RP2040: switched to SDK v2.0.0 and added initial support for RP2350 (Pico 2). The Web Builder will be updated a little later for this change.

Plugins:

* Trinamic: removed superfluous semicolon. Ref. [issue #15](https://github.com/grblHAL/Plugins_motor/issues/15).

* Networking: added some files to _CMakeLists.txt_, used by RP2350 driver.

---

<a name="20241121">Build 20241121

Core:

* downgrading settings reset backup

Plugins:

* Trinamic: removed superfluous semicolon. https://github.com/grblHAL/Plugins_motor/issues/15

---

<a name="20241121">20241121

Core:

* Fixed bug in setting home position to 0 when CoreXY kinematics is enabled. Ref. [ESP32 issue #77](https://github.com/grblHAL/ESP32/issues/77).

---

<a name="20241120">Build 20241120

Core:

* A bit of refactoring related to spindles as a first step for a coming settings structure revision, "hardened" some code.

Drivers:

* Some: added support for ["assorted small plugins"](https://github.com/grblHAL/Plugins_misc/), mainly for drivers available in the Web Builder.

* STM32F1xx: moved board specific code to _board_ directory.

* STM32F4xx: simplified configuration of Trinamic low-level interfaces, fixed typos in second RGB LED channel code
and added workaround for RGB LEDs connected to debug pin beeing turned on at boot.

Plugins:

* Motors: refactored M-code handling and added support for Marlin style `M569` and `M919` commands. Some minor bug fixes.

* Trinamic: fixed "bug" in TMC2660 current handling, extended API.

* Spindle: changed settings handling when multiple spindles are configured. Fixed bug in `M104P<n>` M-code.  
__NOTE:__ an automatic update of settings `$511`-`$513` will be attempted, this may fail so please check them after upgrading.

---


<a name="20241116">Build 20241116

Core:

* Added support for named o-sub/o-call to flow control, for calling gcode subroutines stored on SD card or in littlefs. Sub name matches filename with extension _.macro_.

* Added core event for handling special gcode comments used by expressions: `DEBUG`, `PRINT` and `ABORT`. These can now be extended or new added to by plugins.

* Added overridable default $-setting values for second PWM spindle to _grbl/config.c_.

Plugins:

* Trinamic: changed some default parameter values, fix bug in previously unused TMC2660 code.

* Motors: fixed bugs in handling and visibility of extended settings.

* Misc, eventout: added info to `$pins` command for pins mapped to event actions.

---

<a name="20241113">Build 20241113

Core:

* Added basic core support for toolsetter probe, changes $6 \(probe input inversion\) and $19 \(probe input pullup disable\) settings from boolean to bitfield when driver support is available.

* Added a few new default values for $-settings in _config.h_, overridable from the compiler command line.

* Added core support for per axis pulloff distance, needs plugin for configuring them.

* Added HAL flags for disabling settings for MCU input pins pullup disable, may be set by drivers/boards that has buffered \(optocoupled\) inputs that is not possible to change.

Plugins:

* Spindle: fixed some naming inconsistencies.

* Misc: added plugin for feed rate overrides via Marlin style M-code and plugin for configuring per axis homing pulloff distance.

* Keypad, macros: added some overridable defaults.

---

<a name="20241110">Build 20241110

Core:

* Added generic passthru support for programming external MCU via USB > UART bridge. Requires driver support if to be used. Adds `$PTRGH` system command when available.

Plugins:

* Misc, ESP-AT: general improvements.

---

<a name="20241107">Build 20241107

Core:

* Removed deprecated stream flags, added stream event for line state \(RTS, DTR\) changes - initially for USB streams.

Drivers:

* Many: updated for removal of deprecated stream flags in the core.

* STM32 drivers: added support for line state changed event.

Plugins:

* Trinamic: fix for incorrect min/max stepper current validation for TMC2130 and TMC2209. Ref. STM32F4xx [issue #197](https://github.com/grblHAL/STM32F4xx/issues/197).

* Networking: updated WebSocket daemon for removal of deprecated stream flags in the core.

---

<a name="20241030">20241030

Core:

* Added init call for new ESP-AT plugin.

Plugins:

* Motors: fixed incorrect settings reference. Ref. [discussion #107](https://github.com/grblHAL/ESP32/discussions/107).

* Misc. plugins, esp_at: added experimental support for Telnet over WiFi via an ESP32 running [ESP-AT](https://docs.espressif.com/projects/esp-at/en/latest/esp32/Get_Started/index.html).

---

<a name="20241025">Build 20241025

Core:

* Changed `_vminor` named parameter to return build date in YYMMDD format, previously value was 0.

* Added support for LinuxCNC style `(ABORT,<msg>)` comment, requires expressions enabled.
Terminates gcode program, outputs message and returns error 253.

* Added `PRM[<setting>]` and `PRM[<setting>,<bit>]` functions to expressions, returns $-setting value or value of bit in integer type setting.

* "hardened" flow control code, fixed bug in `repeat...continue` handling.

* Changed signature of `grbl.on_gcode_comment` event, now returns status code.

Drivers:

* ESP32, RP2040: changed NVS storage for settings in flash from 2 to 4KB.

* STM32F4xx: fixed typos.

* Some: added note to _platformio.ino_ file.

Plugins:

* SD Card, macros: fixed G65P1 settings read macro, failed when reading some indexed settings.

---

<a name="20241023">Build 20241023

Core:

* Fixed some odd bugs in NGC flow control, prepared for file based named O-call subroutines.

* Fixed incorrect comment string passed to `grbl.on_gcode_comment` event.

* Added generic redirector for temporarily changing input stream to read from a file. Supports nesting.

Drivers:

* ESP32: fix for overriding UART0 pins, reverted  and fixed tests for ESP32-S3 conditional code.

Plugins:

* SD Card, macros: updated to use new input stream redirector, allows nesting of `G65` calls
 \(max 5 levels depending on available memory\). __NOTE:__ Not extensively tested, feedback required.

* SD card: updated to work alongside new file redirector.

---

<a name="20241019">Build 20241019

Core:

* Moved message substitution code from _gcode.c_ to _ngc_expr.c_.

Drivers:

* ESP32: fixed tests for ESP32-S3 conditional code, added overridable symbols for UART0 RX/TX pins.

* STM32F4xx: fixed regression in spindle sync code. Ref. [discussion #612](https://github.com/grblHAL/core/discussions/612).

Plugins:

* Spindle: fixed name typo. Ref. [issue #34](https://github.com/grblHAL/Plugins_spindle/issues/34).

---

<a name="20241016">Build 20241016

Core:

* Improved parameter handling some more, now allows indirected O-calls and line numbers with O-word.

* Fix for [issue #609](https://github.com/grblHAL/core/issues/609), homing may cause a controller crash.

Plugins:

* Spindle: fixed compiler warning. Ref. [issue #33](https://github.com/grblHAL/Plugins_spindle/issues/33).

---

<a name="20241014">Build 20241014

Core:

* Improved expression and parameter handling, simplified gcode parser related to this. Fixed typo in `OR` statement decode. 

* Added clear of return value on `CALL` statement and added optional return value expression support to `ENDSUB`.

* For developers: moved user M-code entry point hooks from the HAL structure to the core handlers and
changed signature of the _check()_ function to better support valueless words \(letters\) when parameter support is enabled.
Removed deprecated parameter from the _validate()_ call signature.

* Improved handling of multiple simultaneous spindles. Still work in progress.

Drivers: 

* STM32F4xx and STM32F7xx: fixed some typos in new timer API. Ref. [issue #192](https://github.com/grblHAL/STM32F4xx/issues/192)
and [issue #18](https://github.com/grblHAL/STM32F7xx/issues/18).

Plugins and templates:

* Some updated for move of core M-code entry points.

---

<a name="20241006">Build 20241006

Core:

* Fix for `M62` - `M68` regression. Ref. [issue #600](https://github.com/grblHAL/core/issues/600).

* Fixed incorrect handling of `G65` call parameters, axis words had offsets added.  Ref. [issue #594](https://github.com/grblHAL/core/issues/594).

* Refactored handling of multiple spindles. There are still some limitations but should work better now. Disabled override delays for now, needs investigation. Ref. [issue #598](https://github.com/grblHAL/core/issues/598).  
__NOTE:__ Please report any erratic behaviour after installing this version since it is a rather major change.

Drivers:

* ESP32: fix for compilation error. Ref. [issue #122](https://github.com/grblHAL/ESP32/issues/122).  
Fixes for handling multiple devices on a single SPI port. Fixed xPro v5 map for Modbus comms. Ref. [issue #121](https://github.com/grblHAL/ESP32/issues/121).

* STM32F4xx: fix for compilation error for some boards when configured for Trinamic drivers.

Plugins:

* BLTouch: implemented `$BLTEST` command, verified.

---

<a name="20240928">Build 20240928

Core:

* Added `(PRINT, <msg>)` support and parameter formatting for `DEBUG` and `PRINT` commands. Available when expression support is enabled.

* Added named parameters for getting absolute \(G53\) position: `_abs_x`, `abs_y`, ... Available when expression support is enabled.

* Changed stepper enable HAL signature to allow current reduction when idle. Requires compatible stepper drivers and low level code support.

* Added reference id to spindle registration in order to allow configuring default spindle, and possibly additional spindles, at compile time.

* Fix for hardfault when some $-settings are changed and there are no auxilary inputs defined in the board map. Ref. [issue #588](https://github.com/grblHAL/core/issues/588).

Drivers:

* All: updated for core changes mentioned above.

* ESP32, STM32F4xx, STM32F7xx: added basic support for the core HAL timer API. Changed step inject code to interrupt driven instead of polled.

Plugins:

* Spindle: updated to support new spindle reference id in the core, simplified code.

* SD Card \(macros\): fixed bug in handling of repeat loops.

---

<a name="20240921">Build 20240921

Core:

* Added generic HAL timer API and function for getting which `G65` parameter words were supplied. 

Plugins:

* Networking: made parsing of HTTP header keywords case insensitive. Ref. [issue #11](https://github.com/grblHAL/Plugin_networking/issues/11).

* SD card \(macros\): added inbuilt `G65` macro `P3` for getting and setting NGC numerical parameters, typical use case will be for indexed access. Ref. [discussion #309 comment](https://github.com/grblHAL/core/discussions/309#discussioncomment-10710468). 

---

<a name="20240907">Build 20240907

Core:

* Added some RGB LED strip properties, improved handling of single meaning G-code words claimed by user M-codes.

Plugins:

* Misc: updated for new RGB LED strip properties.

Drivers:

* iMRX1062, STM32F4xx and STM32F7xx: updated for new RGB LED strip properties.

* RP2040: revised pin mappings for BTT SKR Pico board. Added misc. plugins to compilation.

---

<a name="20240903">Build 20240903

Core:

* Added some new plugin init calls and setting ids. Added defaults for RGB strip lengths.

Drivers:

* ESP32, RP2040, STM32F4xx and  STM32F7xx: updated for core changes related to the RGB HAL.

* RP2040: renamed bluetooth files to avoid conflict with SDK.

* STM32F7xx: moved board maps to separate directory.

Plugins:

* SD card: removed superfluous code. Made _.macro_ file type visible by default. Ref. [ioSender issue #403](https://github.com/terjeio/ioSender/issues/403).

* Misc: initial commit of [new plugins](https://github.com/grblHAL/Plugins_misc), some moved from [Templates](https://github.com/grblHAL/Templates/tree/master/my_plugin).

* WebUI: now delays soft reset commands for ESP32 driver to avoid crash when more than 3 axes are enabled. Ref. [issue #15](https://github.com/grblHAL/Plugin_WebUI/issues/15)

---

<a name="20240827">Build 20240827

Core:

* Added setting definitions for plugins and some new plugin initialization calls.

Drivers:

* ESP32: changed spindle on signal to GPIO32 when On/Off spindle is configured for MKS DLC32 board. Ref. this [discussion](https://github.com/grblHAL/core/discussions/203#discussioncomment-10454788).

* RP2040: fixed build issues when native Bluetooth is enabled. Ref. [issue #94](https://github.com/grblHAL/RP2040/issues/94).

Plugins:

* Spindle: added "Offset" plugin for spindle \(laser\) movement to be executed when switching between spindles.

* WebUI: workaround for [issue #15](https://github.com/grblHAL/Plugin_WebUI/issues/15), ESP32 crash on soft reset when > 3 axes configured.

* Miscellaneous: added a number of smallish plugins; BLTouch, PWM servo, EventOut, RGB LED strips, RGB LED M150. These are work in progress and requires specific driver configurations.

---

<a name="20240817">Build 20240817

Core:

* Simplified keypad and MPG symbol handling.

Drivers:

* Most: updated for simplified keypad and MPG symbol handling.

* LPC176x: added support for keypad and MPG plugin. I2C keypad not fully supported and not tested.

Plugins:

* Keypad: updated for simplified keypad and MPG symbol handling.

---

<a name="20240812">Build 20240812

Core:

* Improved handling of extended M commands \(plugin based\) command words. Fixes issues for programs containing extended M-codes using single meaning words \(which they as a rule should not do\).

* Added core support for spindle encoder binding to spindles.

* Added sorting of spindle report: enabled spindles are sorted first in order of spindle number, disabled by type then spindle id.

* Changed realtime report to report spindle number instead of spindle id on changes in the `|S:` element. Part of fix for ioSender [issue #399](https://github.com/terjeio/ioSender/issues/399).

Drivers:

* imXRT1061, MSP432, STM32F4xx, STM32F7xx: updated to take advantage of new spindle encoder binding functionality.

Plugins:

* Spindle: updated relevant drivers to use new spindle encoder binding functionality, simplified code. Fixes [issue #30](https://github.com/grblHAL/Plugins_spindle/issues/30).

---

<a name="20240805">Build 20240805

Core:

* Added function for getting speed \(RPM\) of stepper controlled by secondary stepper motor driver.

Plugins:

* Spindle: stepper spindle code now uses new function for getting speed \(RPM\) of motor.
HAL functions for getting spindle data \(actual RPM, angular position etc.\) directed to stepper spindle code,

---

<a name="20240801">Build 20240801

Core:

* Added option bit for enabling realtime reporting while homing to `$10`, _Status report options_. Ref. [issue #551](https://github.com/grblHAL/core/issues/551).  
__NOTE:__ Enabling this may affect some senders.

Drivers:

* iMXRT1062, LPC176x, SAM3X8E and STM32F1xx: moved board maps/board specific code to new _boards_ directory. 

* STM32F4xx: fixed regression in SD card code affecting boards using SDIO interface.

---

<a name="20240719">Build 20240719

Core:

* Limited tool change probe moves to be within machine limits. Ref. [issue #542](https://github.com/grblHAL/core/issues/542).

* Added setting `$538` to enable experimental functionality for fast rotary 'rewind' to stored G28 position. Return move should complete in half a rotation or less.  
To use program:
```
G91G28<axisletter>0
G90
```
where \<axisletter\> is the axisletter of the rotary axis to move.

* For developers: added option to redirect debug via driver provided function when `DEBUGOUT` is set to `-1` in _config.h_. Can be used to send debug output to debugger interfaces.

Plugins:

* WebUI: fix for crash when settings page is opened with macros plugin enabled. Ref. [issue #14](https://github.com/grblHAL/Plugin_WebUI/issues/14).

---

<a name="20240709"/>Build 20240709

Core:

* For developers: added printf style debug output function and corresponding macro. See _grbl/stream.h_ for details. Added `grbl.on_report_ngc_parameters` event.

* Fixed silly mistakes in CAN code. Ref. [issue #179](https://github.com/grblHAL/STM32F4xx/issues/179#issuecomment-2217912406).

Drivers:

* SAM3X8E: [PR #25](https://github.com/grblHAL/SAM3X8E/pull/25), adds missing guards around references.

* STM32F1xx: added tentative board map for Creality v4.4.2 and v4.4.7. Ref. [issue #33](https://github.com/grblHAL/STM32F1xx/issues/33). Not tested!

---

<a name="20240704"/>Build 20240704

Core:

* Added high level CANbus API for plugin use. If driver/board combo provides the required lowlevel HAL API the `NEWOPT` string in the `$I` output will contain the `CAN` element when CAN is enabled.  
Ref. [issue #179](https://github.com/grblHAL/STM32F4xx/issues/179).

* Added soft limits check for corexy kinematics, ref. [discussion #536](https://github.com/grblHAL/core/discussions/536).

Drivers:

* ESP32: fixed WebUI regression. Ref. [issue #116](https://github.com/grblHAL/ESP32/issues/116).

* STM32F7xx, STM32F4xx: added lowlevel CANbus API and enabled it for some boards.

---


<a name="20240624"/>Build 20240624

Core:

* Some minor changes to better support keypad macro and networking plugins.

Drivers:

* ESP32: disabled cycle start input for MKS DLC32 board due to incompatible use. Ref. [issue #111](https://github.com/grblHAL/ESP32/issues/111).

* RP2040: fixed incorrect handling of WiFi BSSID.

* STM32F7xx: completed support for WizNet ethernet modules \(W5100S & W5500\). Added code by @dresco for getting unique MAC address, Ref. [discussion #17](https://github.com/grblHAL/STM32F7xx/discussions/17).

Plugins:

* Networking \(WizNet\): fixed incorrect MAC address handling, added weak functions for getting default MAC addresses ++. Ref. [discussion #17](https://github.com/grblHAL/STM32F7xx/discussions/17).

* Keypad macros: added settings for binding single character realtime commands to macro pin event.  
__NOTE:__ this change will reset _all_ plugin settings to default, backup/restore if this plugin is in use.

---

<a name="20240619"/>Build 20240619

Core:

* Renamed `grbl.on_probe_fixture` event to `grbl.on_probe_toolsetter` and added pointer to toolsetter coordinates as an additional parameter.
This will allow plugin code to modify the coordinates before moving to the toolsetter so that tools with off center cutting surfaces can be properly measured.
Ref. [ioSender issue #386](https://github.com/terjeio/ioSender/issues/386).

* Increased backlash parameters precision to 5 decimals. Ref. [issue #452](https://github.com/grblHAL/core/issues/452#issuecomment-2159050856).

* Some bug fixes in NGC parameters and flow control handling.

Drivers:

* ESP32: improved SPI code.

* SAM3X8E: added support for native and MCP3221 I2C ADCs. Ref. [issue #24](https://github.com/grblHAL/SAM3X8E/issues/24).

Plugins:

* Templates: Updated some for core event rename and signature change.

---

<a name="20240604"/>Build 20240604

Core:

* Fixed incorrect implementation of `EXISTS` function. Ref. [issue #527](https://github.com/grblHAL/core/issues/527).

* Added missing clear of parser tool change state when cycle start signal is asserted. Affects tool change mode 'Normal' ($341=0).

---

<a name="20240602"/>Build 20240602

Core:

* Fixed typo/bug in expression precedence handling. Ref. [issue #527](https://github.com/grblHAL/core/issues/527).

Drivers:

* STM32F4xx, STM32F7xx: added cast to suppress compiler warning.

* STM32F7xx: added DMA support for SPI4, fix for SPI SCK pin reporting. WizNet code still not working.

---

<a name="20240527"/>Build 20240527

Core:

* "Hardened" NGC parameter name case swapping, changed to use float precision according to metric/inches setting for parameter reporting. 

---

<a name="20240526"/>Build 20240526

Core:

* Added experimental support for M70-M73, save and restore of modal state.

* Added experimental support of LinuxCNC style subroutines.
Available for gcode run from local filesystem such as on a SD card or in littlefs.

* Improved handling of G92 when G92 offset is changed while motion is ongoing.
Position and WCO offset in the realtime report will now be the actual realtime values and not
the values based on the parser state which may be quite a bit ahead of the machine. Ref. [discussion #241](https://github.com/grblHAL/core/discussions/241#discussioncomment-9463390).

* Fix for [issue #521](https://github.com/grblHAL/core/issues/521), crash when running G65 macro on ESP32.

* "Hardened" stream switching code, likely fix for [discussion #456](https://github.com/grblHAL/core/discussions/456#discussioncomment-9533613).

Drivers:

* STM32F7xx: added initial support for WizNet W5500 and W5100S ethernet modules. Stack starts up but fails later, likely due to SPI issue?.
To be completed later.

Plugins:

* Keypad, OpenPNP and Encoder: updated for core signature change related to improved G92 handling.

* Networking: fixed incorrect signature of WizNet ethernet init function.

---

<a name="20240513"/>Build 20240513

Core:

* Fix for homing setting regression. Ref. [issue #512](https://github.com/grblHAL/core/issues/512).

Drivers:

* STM32F1xx, STM32F3xx, STM32F4xx: made UART code fully core compliant.

Plugins:

* Networking: fixed ping response when no payload present. Ref. [issue #10](https://github.com/grblHAL/Plugin_networking/issues/10).

---

<a name="20240508"/>Build 20240508

Core:

* Fix for stream regression, improved MPG stream handling. Ref. [issue #509](https://github.com/grblHAL/core/issues/509).

* Added NGC parameter 5599, debug output enabled status.

Drivers:

* ESP32: made UART code fully core compliant, fix for MQTT compilation error.

* STM32F7xx: added missing MPG code, additional I2C port support and made UART code fully core compliant.

---

<a name="20240506"/>Build 20240506

Core:

* Fix for incorrect handling of some flow control statements when nested. Ref. issue [#504](https://github.com/grblHAL/core/issues/504).

* Fixed defaults and added sanity checks for spindle linearization parameters settings.

Drivers:

* ESP32: increased max application size to 2 MB. __NOTE:__ settings and any WebUI files stored in littlefs will be overwritten on an update, backup and restore when updating!

* STM32F1xx: removed stray debug message.

* STM32F4xx: added printf/scanf support to STM32CubeIDE builds with spindle linearization enabled, due to run time issues.  
Fixed incorrect EEPROM emulator flash section id, ref. core [discussion #503](https://github.com/grblHAL/core/discussions/503) and core [issue #457](https://github.com/grblHAL/core/issues/457).

---

<a name="20240427"/>Build 20240427

Core:

* Added config to enable NGC parameter reporting, default on.

Drivers:

* STM32F1xx: disabled NGC parameter reporting in order free up some flash space \(for 128K variants\).

Plugins:

* SD card (file system macros): added inbuilt `G65P2Q<tool>R<axis>` macro for reading tool offset from tool table. `<tool>` is tool number, `<axis>` is axis number: 0 = X, 1 = Y, ...

* Keypad: allow MPG to take control when estop state is active.

---

<a name="20240425"/>Build 20240425

Core:

* Now reports WCO along with radius/diameter mode changes. Ref. issue [#500](https://github.com/grblHAL/core/issues/500).

Drivers:

* STM32F1xx: fix for broken handling of control signals for RC variant processors. Ref. issue [#51](https://github.com/grblHAL/STM32F1xx/issues/51) and discussion [#499](https://github.com/grblHAL/core/discussions/499).

Plugins:

* SD card (file system macros): added inbuilt `G65P1Q<n>` macro for reading numeric setting value. `<n>` is setting number. Ref. issue [#493](https://github.com/grblHAL/core/issues/493).

---

<a name="20240420"/>Build 20240420

Core:

* Fix for bug/compiler warning. Ref. discussion [#492](https://github.com/grblHAL/core/discussions/492).
* Fix for broken initialization of wall plotter machine properties.

---

<a name="20240418"/>20240418

Core:

* Fix for compiler warning.

Drivers:

* ESP32: updated Root CNC v3 map and added Root CNC Pro map. Ref. issue [#102](https://github.com/grblHAL/ESP32/discussions/102).  
Added missing comma. Ref. ioSender issue [#367](https://github.com/terjeio/ioSender/issues/367#issuecomment-2066416027).
Added tentative support for additional I2C API functions.  

* STM32F4xx: removed stray debug message, fixed I2C strobe and MPG mode input handling. Updated FatFS \(SPI\) to use new task scheduler.

* STM32F7xx: fix for compiler warning.

Plugins:

* Keypad (display): workaround for ESP32 compiler complaining about `static_assert`.

Templates:

* Persistent tool: updated for core change.

---

<a name="20240416"/>Build 20240416

Core:

* Fix for random feed hold/cycle start sequence failures. Ref. issue [#491](https://github.com/grblHAL/core/issues/491).

Drivers:

* STM32F1xx: added tentative support for UART4, not tested!

* STM32F7xx: added support for SPI4, not tested!

* Simulator: added support for continuous 1ms systick event. Ref issue [#8](https://github.com/grblHAL/Simulator/issues/8).

---

<a name="20240408"/>Build 20240408

Core:

* Fix for bug in NGC expressions return statement handling. Ref. issue [#485](https://github.com/grblHAL/core/issues/485).

Drivers:

* RP2040: fix for incorrect handling of safety door input inversion. Ref. issue [#85](https://github.com/grblHAL/RP2040/issues/85).

* STM32F7xx: removed stray project folder from Eclipse debug build configuration.

* All \(remaining\): now calls stepper enable via HAL.

Plugins:

* SD card: fix for potential expression stack issue when macro is terminated early with `M99`.

* Spindle: improved logic, switched polling to new task handling code. Ref. issue [#27](https://github.com/grblHAL/Plugins_spindle/issues/27).

---

<a name="20240404"/>Build 20240404

Core:

* Fixed polar kinematics feed rate handling, some tuning. Ref. issue [#475](https://github.com/grblHAL/core/issues/475).

* Allowed plugins to inject commands when controller is in alarm state. Ref. keypad plugin issue [#11](https://github.com/grblHAL/Plugin_keypad/issues/11).

Drivers:

* STM32F1xx, STM32F3xx, STM32F4xx, STM32F7xx and iMXRT1062: now calls stepper enable via HAL. Ref. spindle issue [#28](https://github.com/grblHAL/Plugins_spindle/issues/28).

Plugins:

* Spindle: improved motor enable support for stepper spindle. Ref. issue [#28](https://github.com/grblHAL/Plugins_spindle/issues/28).

* Plasma: removed stray code causing compilation failure.

---

<a name="20240402"/>Build 20240402

Core:

* Fixed symbol issue with Arduino Due blocking compilation of NGC parameter support.

Drivers:

* SAM3X8E: added driver support for additional aux inputs in order to support probe input.

* iMXRT1062: fixed typo in step inject code causing direction signal to fail for A+ axes.

Plugins:

* Spindle: added motor enable support for stepper spindle.

---

<a name="20240330"/>Build 20240330

Core:

* Added capability flags to HAL for all coolant outputs.

* Hide related settings when no spindle and/or coolant outputs are available. From [PR #479](https://github.com/grblHAL/core/pull/479).

* Fixed typo related to Modbus direction signal. Ref. issue [#478](https://github.com/grblHAL/core/issues/478).

* Fixed typo in handling of aux output port settings. Ref issue [#476](https://github.com/grblHAL/core/issues/476).

Drivers:

* All: updated for coolant capability flags core change.

Plugins:

* Spindle: added packet length to Modbus RX callback. Ref. [issue #26](https://github.com/grblHAL/Plugins_spindle/issues/26).

* Macros: fixed typo causing compilation to fail.

* Trinamic: switched to enum for default driver mode to avoid warnings from TI compiler.

---

<a name="20240328"/>Build 20240328

Core:

* Added missing null spindle handler for ESP32, issue [#473](https://github.com/grblHAL/core/issues/473).

* Fix for unable to set $484 to 0, issue [#466](https://github.com/grblHAL/core/issues/466).

* Added setting $673 for setting coolant on delay after feedhold. Available when safety door handling is not enabled.
Fixed obscure bug carried over from legacy Grbl related to this. Issue [#467](https://github.com/grblHAL/core/issues/467).

* Enabled setting $394 for spindle on delay after feedhold. Available when safety door handling is not enabled.

Drivers:

* RP2040: fixed regression causing step generation for BTT SKR Pico to partly fail in some configurations. "Hardened" code.

Plugins:

* Motors: fixed bug that left ganged motor drivers in wrong state after leaving the ioSender _Trinamic tuner_ tab.

---

<a name="20240326"/>Build 20240326

Core:

* Fixes for issue [#470](https://github.com/grblHAL/core/issues/470) and [#472](https://github.com/grblHAL/core/issues/472), index overflows.

Drivers:

* iMXRT1062: added support for second UART stream and missing code guard for I2C strobe.

* STM32F4xx and STM32F7xx: improved analog I/O, added check for multiple calls to I2C initialization.

---

<a name="20240318"/>Build 20240318

Core:

* Changed signature of `grbl.on_homing complete` event to include the cycle flags.

* Added definitions for up to four additional digital aux I/O ports.

* Added real time report of selected spindle in multi spindle configurations. Reported on changes only. 

* Removed limits override input invert config, for safety reasons it is always active low.

* Added HAL entry points for second RGB channel, renamed first from `hal.rgb` to `hal.rgb0`.

* Added option to setting `$22` to force use of limit switches for homing when homing inputs are available in the driver/board combo.

* Improved handling of aux I/O pins when claimed for core functions.

* Added flag to `$9` for disabling laser mode capability for primary PWM spindle. Allows leaving laser mode enabled when a
secondary PWM spindle is available and this is used to control a laser.

* Added some generic setting definitions for stepper drivers, currently used by the motors plugin.

Drivers:

* STM32F4xx: added support for secondary PWM spindle, home signal inputs and preliminary support for the Sienci SLB board. Moved PWM spindle code to separate file.  
Added full WebUI support for F412 and F429 MCUs, others may work but with settings missing due to limited RAM.  
Merged some code/adopted ideas from the Sienci SLB project for bitbanged Neopixel support and `$DFU` command for entering DFU bootloader mode \(via USB only for now\).

* STM32F7xx: added support for secondary PWM spindle. Moved PWM spindle code to separate file. Added tentative support for F765 MCU. Some PWM bug fixes.

* ESP32: added 6pack v2 board, fixes for v1 definitions - from [PR #103](https://github.com/grblHAL/ESP32/pull/103). Added driver support for max limit switches and missing init for e-stop input.

* iMXRT1062, STM32F4xx, STM32F7xx and MSP432: updated for core change related to spindle at speed handling.

* RP2040: fix for [issue #83](https://github.com/grblHAL/RP2040/issues/83), missing code guards.

* iMXRT1062, STM32F4xx, RP2040 and ESP32: updated for core change related to the RGB HAL.

Plugins:

* Motors: updated to match improved Trinamic low level driver API. Added optional settings for some tuning parameters and made default symbols overridable.
Merged some code/adopted ideas from the Sienci SLB project for new functionality.
Added option for routing StallGuard signals to homing inputs when supported by the driver.  
__NOTE:__ some of these changes has only been lightly tested.

* Spindle: updated for core changes, switched to shared code for PWM spindle configuration.

* Keypad: limited code guard for enabling keypad plugin to 1 - 3 range to allow other implementations.

* Plasma: corrected messed up settings handling.

* Trinamic drivers: improved configuration handling/inheritance, adapted support for TMC2660 from the Sienci SLB project. Some generic changes to improve consistencty among drivers.

---

<a name="20240228"/>Build 20240228

Core: 

* Tentative fix for lathe CSS/feed per rev modes \(G96/G95\), ref discussion [#450](https://github.com/grblHAL/core/discussions/450).

* Fixed incorrect reporting of feed rate modes in `$G` response.

* Added some ioPorts call wrappers, full support for reconfiguring output auxillary pins for PWM output.

Drivers:

* STM32F1xx, ST32F4xx, STM32F7xx: fix for incorrect status returned from ioPort configuration call.

* STM32F7xx: added full support for reconfiguring capable auxillary output pins for PWM.

Plugins:

* WebUI: added update that did not make it in the previous one...

---

<a name="20240226"/>Build 20240226

Core: 

* Fixed typo preventing lathe mode UVW builds.

* Added initial support for using auxillary output pin pool for spindle and coolant pins.

Drivers:

* STM32F7xx: updated to use auxillary output pin pool for spindle pins. Enhanced and simplified PWM support, added generic Uno and Protoneer v3 boards to Web Builder.

Plugins:

* WebUI: "hardened" code a bit more in attempt to make it usable with STM32F4xx driver which has limited RAM that blocks reporting settings data.

---

<a name="20240222"/>Build 20240222

__NOTE:__ This build has moved the probe input to the ioPorts pool of inputs and will be allocated from it when configured.  
The change is major and _potentially dangerous_, it may damage your probe, so please _verify correct operation_ after installing this build.  
There were basically two reasons for doing this, one is to free the input for general use if not needed and another is to be able to add advanced
probe protection support, via plugin code, for probe inputs that has interrupt capability.

Core:

* Fix for STM32F4xx [issue #161](https://github.com/grblHAL/STM32F4xx/issues/161), Ethernet connection unresponsive if USB port not powered.

* Enhanced ioPorts interface: New debounce option for input pins that are interrupt capable, currently only possible to enable via plugin code - later to be made available via a $-setting.  
Added some wrapper functions for simpler plugin code++


* Added simple task scheduler to the core, allows interrupt routines to dispatch jobs to the foreground process, delayed tasks and repeating tasks attached to the 1 ms system timer.  
Some drivers and plugins now uses the scheduler for input pin debouncing, regular polling etc. The core uses it for stepper disabling and sleep monitoring.

Drivers:

* Most: updated to support the enhanced ioPorts interface and using the new task scheduler for debouncing etc.
  Moved probe and safety door inputs to ioPorts pin pool, if not assigned at compile time they will be free to use by M66 or plugin code.   
  Many board maps has been updated to take advantage of the new ioPorts capabilities. PLease report anny irregularities as I do not have access to all the boards for testing.

* ESP32: MKS Tinybee v1.0 map changed to use MT_DET \(pin 35\) for motor 3 limit input \(e.g. auto squared Y\) if probe input is not enabled. Ref. [issue #93}(https://github.com/grblHAL/ESP32/issues/93#issuecomment-1917467402).

Plugins:

* Some: updated to use the new task scheduler for polling etc.

* Networking: WizNet driver code updated to use the new task scheduler for interrupt handling, resulting in faster/smoother operation.

* WebUI: a bit of code "hardening", reduced memory requirement for some of the larger generated messages.

---

<a name="20240205"/>Build 20240205

Core:

* Added core support for new MPG mode that claims one serial stream and uses the `0x8B` real-time command character for switching mode. Does not require the keypad plugin.

* Moved RGB API definitions to separate file and added some utilities for drivers and plugins. Fixed minor bug.

* For developers: `stream_open_instance()`, signature change - added optional description string.

Drivers:

* Many: updated to support new MPG mode. Updated for core signature change.

* ESP32, RP2040, STM32F4xx: enhanced Neopixel support. __Note:__ Not yet used by any boards.

* STM32F7xx: added missing MPG mode handlers.

Plugins:

* Bluetooth: updated for core signature change.

---

<a name="20240202"/>20240202

Core:

* Fixed compiler warning for compatibility level > 1, added some "just in case" assertions.

Drivers:

* Most: "hardened" ioports code to avoid hardfault in some configurations.

---

<a name="20240131"/>20240131

Drivers:

* iMXRT1062: fixed regression causing spindle sync builds to fail.

* STM32F4xx: moved board maps and code to separate directory. Fixed minor bug.

* Web Builder supported boards: updated board definition files for Web Builder internal changes.

---

<a name="20240129"/>20240129

Drivers:

* ESP32: made usage of second UART/serial port more flexible for those boards that has support for it.

* iMXRT1062: fixed regression blocking builds with the Bluetooth plugin enabled. Ref. Bluetooth plugin [issue #4](https://github.com/grblHAL/Plugins_Bluetooth/issues/4).

---

<a name="20240127"/>Build 20240127

Core:

* Fixed regression mainly affecting WebUI enabled builds. Some internal changes.

Drivers:

* ESP32: reorganized main driver code for readability, minor fixes.

Plugins:

* SD card: commented out unused code to avoid compiler warning.

---

<a name="20240125"/>Build 20240125

Core, for developers:

* Simplified [regstration](http://svn.io-engineering.com/grblHAL/html/system_8c.html#a480cedc4c3840cfebb4d5fdce898dd3b) of additional system commands, deprecated original method.
* Added [improved call](http://svn.io-engineering.com/grblHAL/html/protocol_8c.html#a869dff1f8d0b3965578eb3e6a94729c1) for registering single run tasks to be executed in the foreground, deprecated [original call](http://svn.io-engineering.com/grblHAL/html/protocol_8c.html#a78fa9a198df36192acec52d28d760a6c).
* Moved VFS events from `grbl.*` to [vfs.*](http://svn.io-engineering.com/grblHAL/html/vfs_8h.html#ab87cc94daec156bea722f9f05d7eeb0c).

Drivers:

* Most: updated to use new method for registering single run tasks. Updated ioports code for improved core compliance.

* LPC176x: fix for [issue #44](https://github.com/grblHAL/LPC176x/issues/44), non-existing probe pin.

Plugins:

* Many: updated to use new method for registering single run tasks. Some bug fixes.

---

<a name="20240123"/>Build 20240123

Core:

* Enhanced RGB and ioports APIs.

* Allowed use of unused \(by the core\) axis words \(ABCUVW\) in M-code commands implemented by plugin code.

* Added `$PINSTATE` command, for outputting auxillary pin states, modes and capabilities. Machine readable formatting.

Drivers:

* RP2040: refactored allocation/initialization of PIO state machines to allow a neopixel driver, and possibly user drivers, to be installed.

* ESP32, RP2040, STM32F4xx: added Neopixel driver code exposed via the core RGB API.  
__NOTE:__ there is no official board support for this just yet.

* Some: added full or partial support for new features in the ioports API.  
__NOTE:__ This work is not yet complete, final tuning and update of remaining drivers will be done later.

Plugins:

* Motors: added overridable symbol for specifying R sense for Trinamic drivers.

Templates:

* Added plugin for RGB LED control via [M150](https://marlinfw.org/docs/gcode/M150.html), sits on top of the new RGB API.

Web Builder:

* Added new tab for assigning optional and dedicated inputs to auxillary inputs.  
Some internal changes to simplify board specifications. Currently the ESP32 and STM32F1xx divers has been updated for this, more to follow later.  
__NOTE:__ Please report any unwanted/unexpected change in behaviour of the generated firmware.

For info:

I have plugings in the pipeline for PWM servo control via [M280](https://marlinfw.org/docs/gcode/M280.html) plus automatic BLTouch probe deployment.  
These are based on original work by @wakass and might be published by him if a PR I plan to submit is accepted.

---

<a name="20240118"/>Build 20240118

Core:

* Added RGB API to the HAL, with crossbar definitions for number of devices \(LEDs, NeoPixels\).

* Refactored some inconsistent parts of the ioports/crossbar interfaces.  
For developers: the signature of the [ioports_enumerate()](http://svn.io-engineering.com/grblHAL/html/ioports_8c.html#ae46c4f9a7ebeac80607a3015da5ab412) call has been changed.
Added PWM servo capability.

Drivers:

* Most: updated for the changed ioports/crossbar interfaces.

* ESP32: added NeoPixel driver for the new RGB API. _Experimental, no board support yet._  
Added auxillary analog PWM out option for up to two channels. Can be configured for PWM servos.

* RP2040: added NeoPixel driver for the new RGB API. _Experimental and untested, no board support yet._

Plugins:

* Spindle: fix for [issue #24](https://github.com/grblHAL/Plugins_spindle/issues/24) - typo, regression causing compilation failure for the MODVFD driver.

* Keypad: I2C display interface, [issue #9](https://github.com/grblHAL/Plugin_keypad/issues/9) - missing update to match core changes caused compilation failure.

---

<a name="20240115"/>Build 20240115

Core:

* Updated build date and a minor insignificant fix.

Drivers:

* ESP32: switched to low-level I/O register access for speed, added WS2812 RGB HAL, completed 32 \(or 31?\) bit I2S shift register output for ESP32-S3, 16 bit not yet ready.

* STM32F7xx: added missing code for ganged axis support \(auto squaring was not affected\).

---

<a name="20240109"/>Build 20240109

Core:

* Fix for issue #426, decreasing the `$30` setting value \(max spindle RPM\) causes incorrect PWM output.

* Implemented handling of _single block_, _block delete_ and _optional stop disable_ control signal events.  
Added help for `$S` \(single block\), `$B` \(block delete\) and `$O` \(optional stop disable\) commands that can be used
to toggle the functionality when the corresponding switch inputs are not available.

* Added [optional HAL entry point](http://svn.io-engineering.com/grblHAL/html/structrgb__ptr__t.html) for outputting WRGB values to lights such as neopixels. The WRGB API may be extended later.

* Improved $-commands registration to make it easier to add help text and keep it in sync. Added missing help text for some core commands.  
Tip: send `$help commands` to output command help.

Drivers:

* ESP32: moved build configuration from [CMakeLists.txt](https://github.com/grblHAL/ESP32/blob/master/main/CMakeLists.txt) to
[my_machine.h](https://github.com/grblHAL/ESP32/blob/master/main/my_machine.h), mainly for reducing build time in the Web Builder.  
More changes for ESP32-S3 compatibility, still work in progress but getting closer.  
Added support for third UART \(serial port\).

* SAM3X8E \(Due\): fix for incorrect pin map, [issue #20](https://github.com/grblHAL/SAM3X8E/issues/20).

* STM32F1xx: fix for [issue #47](https://github.com/grblHAL/STM32F1xx/issues/47) - bad linker file.  
__NOTE:__ this moves settings storage to the end of available flash, if updating backup and restore your settings!

* Most: further updates for assigning optional signals to aux input ports - should be working now.  
__NOTE:__ some signals requires aux pins that support pin change interrupt.

Plugins:

* SD card: updated for new $-commands registration \(help text\).

---

<a name="20231229"/>Build 20231229

Core:

* Fixed regression that prevented builds with compatibiliy level set > 1.

* Added `[SIGNALS:xxx]` element to the `$I` output, `xxx` uses the same letters as the `Pn:`
element in the [real time report](https://github.com/grblHAL/core/wiki/Report-extensions#realtime-report) and list all available from the controller except limits inputs.

Drivers:

* Most: added new configuration options in _my_machine.h_ for assigning optional signals to aux input ports. These are:
1. `SAFETY_DOOR_ENABLE` - bound to aux port in the map file for backwards comaptibility.
2. `MOTOR_FAULT_ENABLE` - bound to aux port in the map file since a single MCU input might be routed to several external inputs.
3. `MOTOR_WARNING_ENABLE` - bound to aux port in the map file since a single MCU input might be routed to several external inputs.
4. `PROBE_DISCONNECT_ENABLE` - assigned from unused ports.
5. `STOP_DISABLE_ENABLE` - assigned from unused ports.
6. `BLOCK_DELETE_ENABLE` - assigned from unused ports.
7. `SINGLE_BLOCK_ENABLE` - assigned from unused ports.
8. `LIMITS_OVERRIDE_ENABLE` - assigned from unused ports. Always active low.

If too many inputs are enabled assignment will fail silently for those who cannot be bound.  
__NOTE:__ core functionality for some of these inputs might change after user input!  
Tip: use the `$pins` command to output the mapping.

* ESP32: moved board maps and board specific code to separate folder.

---

<a name="20231226"/>20231226

Core:

* Added setting and data field for network interface MAC address \(for WizNet interfaces, not used by the core\).

Drivers:

* ESP32: refactored UART driver code to use framework provided low-level calls instead of direct MCU register access.

* RP2040: placed some WizNet interface code in RAM to improve performance.

* STM32F4xx: removed stray debug code.  
Changed pin allocation for WizNet ethernet over SPI for BTT SKR 2.0 board.
Ref. core [discussion #415](https://github.com/grblHAL/core/discussions/415#).

Plugins:

* Networking: improved WizNet interrupt handling.   
Added new optional setting, `$535`, for configuring WizNet interface MAC address.
This _must_ be set to all but one when more than one WizNet controller is added to the network.  
Tip: grab a MAC address from an unused device such as a router.  
__NOTE:__ Network settings for controllers having a WizNet interface will be reset to default on upgrade.

---

<a name="20231222"/>Build 20231222

Core:

* Some minor fixes and changes.

Drivers:

* STM32F1xx: made timer assignments more flexible. Added support for PWM out on PB9. Ref. [core discussion 62](https://github.com/grblHAL/core/discussions/62).

* STM32F4xx: fixed typos in SPI interface code for SPI3.

Plugins:

* Spindle: updated and verified code for second PWM spindle on top of aux outputs. Requires one PWM capable analog output and at least one digital.

* Embroidery: no longer starts spindle if sync mode is disabled.

---

<a name="20231218"/>Build 20231218

Core:

* VFS: fix for incorrect handling of mode flags when mounting root filesystem. Added option to list hidden filesystems.

Drivers:

* ESP32: moved mount of littlefs filesystem until after settings are loaded.

Plugins:

* WebUI: improved handling of hidden littlefs filesystem.

---

<a name="20231217"/>20231217

Drivers:

* ESP32: reverted MKS DLC32 SD card SPI pins assignment, ref. [issue 88](https://github.com/grblHAL/ESP32/issues/88).  
Fixed I2S stepping issues, added dir > step delay with 4 microseconds minimum delay. Ref. [issue 87](https://github.com/grblHAL/ESP32/issues/87).

Plugins:

* Motors: fixed some typos, ref. core [issue 381](https://github.com/grblHAL/core/issues/381#issuecomment-1859083977).

---

<a name="20231216"/>Build 20231216

Core:

* Refactored canned cycles to better match how LinuxCNC actually implements them. Ref. [ioSender issue 347](https://github.com/terjeio/ioSender/issues/347).  
__NOTE:__ The implementation may still be incorrect - use with care!

Drivers:

* SAM3X8E: added laser plugins.

* LPC176x: added laser and spindle plugins.

---

<a name="20231210"/>Build 20231210

Core:

* Spindle handling refactoring for improved management and configuration of multiple spindles.  
__NOTE:__ this is a relatively large change and may have introduced bugs and/or unintended side-effects. Please report any issues!

* Added setting `$519` for binding spindle encoder to given spindle in multi spindle configurations.

* Added machine readable spindle enumeration report, `$SPINDLESH`.

* Increased _default_ value for setting `$398` \(number of planner blocs\) from 35 to 100 for faster laser engraving.
Ref. [this discussion](https://github.com/grblHAL/core/discussions/402).  
__NOTE:__ the `$398` setting value will _not_ change on an upgrade!  
__NOTE:__ STM32F103 builds for the 128K flash variants does not have enough free RAM and will keep 35 as the default value.

* Increased allowed number of decimal places from 3 to 5 for `$10x` stepper step/mm settings.
Ref. [ioSender issue 346](https://github.com/terjeio/ioSender/issues/346).  

* Added setting `$650` for filing system options. Ref. [issue 397](https://github.com/grblHAL/core/issues/397).   
Currently the following bits are available \(depending on the configuration\):  
0 - Auto mount SD card on startup \(1\).  
1 - Do not add littlefs files when listing the root directory \(2\).

* Added build option for [lathe UVW mode](https://www.cnctrainingcentre.com/haas-turn/u-and-w-on-a-cnc-lathe-incremental-programming/).
Ref [this discussion](https://github.com/grblHAL/core/discussions/398).  
When enabled `UVW` words can be used to command relative moves for `XYZ` without switching to relative mode with `G91`. `U` -> `X`, `V` -> `Y`, `W` -> `Z`.  
__NOTE:__ This permanently sets lathe mode and disables the `$32` mode setting.

For developers: 

* There are signature changes to some spindle, ioports enumeration and VFS filing system mount functions.

* Added events to allow plugin code to handle tool table data, possibly stored on a SD card. Ref. [this discussion](https://github.com/grblHAL/core/discussions/392).

Drivers:

* Most: updated for refactored spindle handling and configuration.

* ESP32: fix for spindle at speed failure. Ref. [ioSender issue 345](https://github.com/terjeio/ioSender/issues/345).
Initial changes for ESP32-S3 support and some code refactoring.

* STM32F7xx: fix to reduce stepper current surge on startup. Ref. [issue 400](https://github.com/grblHAL/core/issues/400).

* iMRX1062: fix for hardfault when enabling aux input IRQ early in startup sequence.  Ref. [issue 395](https://github.com/grblHAL/core/issues/395).

Plugins:

* Spindle: updated for core changes. Added several spindles:  
_Nowforever VFD._ \(untested\).  
_Stepper spindle._ This claims the stepper driver from the last configured axis.  
_PWM clone._ This clones the default driver implemented PWM spindle and changes it to use the direction signal for on/off control.  
Settings `$730`, `$731` and `$734` - `$736` will be used to configure the clone. These has the same function as the `$30` - `$36` counterparts.  
The driver spindle is suitable for controlling a laser when `$32` = `1` and the clone is suitable for controlling a spindle motor.  
Switching between the spindles is typically done with `M104Q<n>` where `<n>` is the spindle number.  
_Basic spindle._ Needs and claims 1 or 2 auxillary digital output ports depending on the configuration.  
_Additional PWM spindle._ Needs and claims 1 or 2 auxillary digital output ports and one analog PWM capable port.  

* Motors: fixed default Trinamic motor current - was incorrectly set to 0, changed to 500 mA RMS. Ref. [issue 400](https://github.com/grblHAL/core/issues/400).

* Various: updated for core call signature changes.

---

<a name="20231005"/>Build 20231005

Core:

* Extended secondary stepper driver code and improved debug stream handling.

Drivers:

* iMXRT1062: refactored timer code, improved step injection support.

* STM32F4xx: fixed typo in Trinamic soft serial code.

Plugins:

* Plasma: changed to use rapid rate for THC cutter motion.

* Keypad: increased delay before probing display I2C address to 510ms, [issue #8](https://github.com/grblHAL/Plugin_keypad/issues/8).

* Spindle: added experimental support for stepper spindle. Note: not yet enabled for compilation.

---

<a name="20231002"/>Build 20231002

Core:

* Fixed some typos that may cause compiler warnings.

* Added initial secondary stepper driver code to be used for plasma THC and possibly later for stepper driven spindles.

Drivers:

* ESP32 and RP2040: refactored serial \(UART\) driver code.

* iMXRT1062: improved I2C probe function.

* STM32F1xx, STM32F4xx and STM32F7xx: simplified UART code, added guard to suppress compiler warning for some configurations.

Plugins:

* Plasma: enabled PID loop for voltage THC control.

* Keypad: added 10ms delay before probing display I2C address, [issue #8](https://github.com/grblHAL/Plugin_keypad/issues/8).

---

<a name="20230926"/>Build 20230926

Core:

* "hardened" serial stream registration, now allows empty descriptor.

* now returns no data for alarm and error enums when firmware is compiled for 128K versions of STM32F1xx MCU.
This frees up some flash space for plugins etc.

Drivers:

* STM32F1xx: refactored serial \(UART\) driver code.
All ports enabled by a board is now registered with the core at startup and can be claimed by plugin code if unused. 

* STM32F4xx: refactored serial \(UART\) driver code and added option for 3rd port/stream.
All ports enabled by a board is now registered with the core at startup and can be claimed by plugin code if unused.  
Switched to DMA for SD card transfers in SPI mode and increased clock frequency.
Added tentative board map for MKS Robin Nano v3.

* STM32F7xx: refactored serial \(UART\) driver code and added option for 3rd port/stream.
All ports enabled by a board is now registered with the core at startup and can be claimed by plugin code if unused.  
Added step injection code for plasma plugin.

* iMXRT1062: extended step injection code for plasma plugin.

Plugins:

* Spindle: for developers; harmonized Modbus serial stream selection symbol, old symbol name retained and marked as deprecated.

* Plasma: some minor bug fixes, added check for presence of driver step injection code.

* Keypad: added 10ms delay before probing display I2C address.

---

<a name="20230919"/>Build 20230919

Core:

* Added setting option to _$21 - Hard limit enable_ to enable exception for rotary axes.

Plugins:

* Spindle: fix for [issue #22](https://github.com/grblHAL/Plugins_spindle/issues/22), H100 VFD driver not working.

Drivers:

* Simulator: updates for core changes and for using native libraries for Windows build. Linux and Windows executables can now be built using the [Web Builder](http://svn.io-engineering.com:8080/?driver=Simulator).

* STM32F1xx, STM32F4xx, LPC176x, RP2040: now keeps motor enable signals high until startup is completed to avoid current surges with Trinamic drivers.
Ref [RP2040 issue #74](https://github.com/grblHAL/RP2040/issues/74)

---

<a name="20230917"/>Build 20230917

Core:

* Added setting `$486` that allows locking `G59.1` - `G59.3` coordinate system offsets against accidental changes. Use `$$=386` to list bitmask values.

Drivers:

* iMXRT1062: fix for plasma plugin [issue #2](https://github.com/grblHAL/Plugin_plasma/issues/2#issuecomment-1722313452): MCP3221 ADC driver code not allowing plugin code to claim the input.

Plugins:

* Plasma: made code more robust related to driver configuration changes. Still work in progress!

---

<a name="20230913"/>Build 20230913

Core:

* Added faster soft limits check for arcs when work envelope is a cuboid, fixed some related obscure bugs. New/modified core entry points for soft limit checking.

* More delta robot tuning, added setting warning in `$DELTA` output if some settings are inconsistent such as steps/rad, acceleration etc.

* Fix for issue [#361](https://github.com/grblHAL/core/issues/361), `HOME` status is not reported or reported late when auto reporting is enabled with setting `$481`.

Drivers:

* iMXRT1062: fix for MCP3221 ADC regression, [issue #68](https://github.com/grblHAL/iMXRT1062/issues/68#).

* STM32F4xx: SPI pin definition typo, [issue #139](https://github.com/grblHAL/STM32F4xx/issues/139).

* RP2040: workaround for [issue #74](https://github.com/grblHAL/RP2040/issues/74), odd TMC driver addressing.

* Remaining drivers updated for [improved handling of limit inputs](#20230903).

<a name="20230907"/>Build 20230907

Core:

* Delta robot kinematics tuning: soft limits checks, extended `$DELTA` command++. Still work in progress \(getting closer to working version\).

Plugins:

* Laser: LaserBurn clusters plugin regression fix, new format handling of end-of-line characters.

---

<a name="20230906"/>20230906

Plugins:

* Laser: LaserBurn clusters plugin updated for [new format variation encountered](https://github.com/grblHAL/ESP32/issues/77#issuecomment-1707125447).

---

<a name="20230905"/>Build 20230905

Core:

* The method for constraining continuous multi-axis jogs \(XYZ\) \(when enabled by `$40=1`\) will now use clipping of the motion vector to the work envelope instead of limiting the target to min/max separately for each axis.
The signature of the new core handler introduced in build 20230903 for constraining jog motions has been changed to accomodate this.

* Delta robot kinematics tuning, added new setting for max allowed arm angle. Constraining jog movements to the full work envelope may now work \(I do not have a machine to test with\).
Still work in progress.

Drivers:

ESP32: fix for Modbus RTU serial port board definitions.

STM32F4xx: removed unused code.
 
---

<a name="20230903"/>Build 20230903

Core:

* Changed handling of homing inputs from limit switches. Some drivers will now only disable hard limits (if enabled) for axes that are homing, this includes max/min limit switches.
If max limit switches are available for the board/configuration these will be picked for homing in the positive direction and min switches in the negative direction.
The "unused" limit switches may have hard limits still enabled - depending on the driver.  
__NOTE:__ I plan to add full support for all drivers to keep hard limits enabled for limit switches that are not used for the running homing cycle, this may take some time though as the changes has to be verified.  
__NOTE:__ !! This is a __potentiallly dangerous change___, be careful when homing the machine for the first time after installing/upgrading.

* HAL entry points and core handlers/events has been added and some have changed signatures in order to better support kinematics implementations.

* More work on delta kinematics: new and changed settings, some improved functionality. Still in progress.

Drivers:

* Most: updated for HAL/core event signature changes.

* STM32F4xx: "hardened" Trinamic soft UART code to improve reliability. Added fans plugin.

Plugins:

* Motors: fixed bug that would cause a hard fault if the X driver is not configured as Trinamic when others are. Updated for core event signature change.

* Plasma: relaxed I/O requirements, will now start if up/down signals or arc voltage input is available. Still work in progress.

* SD card: added option for named pallet shuttle macro to be called on M60.

---

<a name="20230825"/>Build 20230825

Core:

* Added `grbl.on_tool_changed` event and removed help for disabled $-commands.

Drivers:

* LPC176x: added option to use reset input as E-stop, added support for Bluetooth plugin for some boards and fixed ioports IRQ issue.

Templates:

* Added _my_plugin_ example for keeping last selected tool number across a reboot. Enable by setting `$485=1`.

---

<a name="20230821-2"/>20230821-2

Plugins:

* Networking: increased WizChip temp packet buffer size, "hardened" code.

Drivers:

* RP2040: use DMA for FatFs transfers, increased WizChip SPI clock to 33 MHz.

* ESP32: updated for change of `grbl.on_homing_completed` signature.

---

<a name="20230821"/>Build 20230821

Core:

* Fix for issue #349 - active feed rate override caused jog motions to hang on some drivers \(STM32F4xx and possibly others\).

Drivers:

* RP2040: fix for regression: ethernet was incorrectly left enabled in CMakeLists.txt.

---

<a name="20230820"/>Build 20230820

Core:

* Delta kinematics improvements. Added setting for base > floor distance, `$DELTA` command for work envelope info. Still WIP.

* Changed signature of `grbl.on_homing_completed` event.

Drivers:

* RP2040: fix for [issue #72](https://github.com/grblHAL/RP2040/discussions/72) \(typo\), improved SPI chip select handling.

---

<a name="20230818"/>Build 20230818

Core:

* Moved kinematics implementations to separate folder and added initial implementation of delta and polar kinematics.  
__NOTE:__ Delta and polar kinematics is WIP \(work in progress\) and incomplete. Feedback is required as I do not have machines at hand for testing.
Ref. issue #341 and #346.

---

<a name="20230816"/>20230816

Plugins:

* WebUI: updated to handle blank commands, used to clear any repeating grblHAL errors. Needs the [latest WebUI build](https://github.com/luc-github/ESP3D-WEBUI/issues/364).

---

<a name="20230815"/>Build 20230815

Core:

* Added setting $484, _Unlock required after E-stop cleared_ by reset, default on. From issue #337.

* Fixed typo, added event `grbl.on_parser_init`.

Drivers:

* Many: replaced parts of aux I/O driver code with calls to shared code in core. Digital aux input inversion bug fix.

* ESP32: fix for incorrect Web Builder option handling for macros plugin.

Templates:

* Added plugin for selecting secondary probe input connected to aux input.

---

<a name="20230810"/>Build 20230810

Core:

* Fix for issue #340, CoreXY kinematics does not update position for axes A+.

* Adds auto reporting state to realtime report sent on MPG mode change to off.

Drivers:

* RP2040: fix for [issue #70](https://github.com/grblHAL/RP2040/issues/70), incorrect handling of I2C/SPI interrupt claims. 

* Many: updated EEPROM option definition to select capacity by Kbits for 1:1 match with chip marking.

---

<a name="20230809"/>20230809

Drivers:

* ESP32: fix for compilation error when VFD is the only spindle specified.

Plugins:

* Many: added CMakeLists.txt for RP2040 builds.

---

<a name="20230808"/>Build 20230808

Core:

* More fixes for issue #332: setting tool table data cleared current coordinate system offset, incomplete handling of G10 L10 and L11.

* Added free memory to $I output when available, example: `[FREE MEMORY:102K]`

* Changed reported position for failed probe to target. Parameters #5061 - #5069 returns position in coordinate system used when probing.

Drivers:

* RP2040: added fans plugin.

* STM32F4xx: implemented free memory call.

Plugins:

* Fans: fixed some bugs and typos.

---

<a name="20230805"/> Build 20230805

Core:

* Fix for issue #332, incorrect NGC parameter values returned for last probed position.

Drivers:

* RP2040: updated PicoCNC board map++ for WizNet module based ethernet support.

* STM32F4xx: tentative fix for [issue #131](https://github.com/grblHAL/STM32F4xx/issues/131), Fysetc S6 board hangs when TMC2209 drivers are enabled. Untested.

Plugins:

* Spindle: fix for [issue #21](https://github.com/grblHAL/Plugins_spindle/issues/21#issuecomment-1660692589), missing VFD settings.

* EEPROM: added option to select capacity by Kbits for 1:1 match with chip marking.

---

<a name="20230729"/>Build 20230729

Core:

* Fix for ioSender issue [#319](https://github.com/terjeio/ioSender/issues/319), improved handling of cycle start input signal had side-effects.

---

<a name="20230724"/>Build 20230724

Core:

* Fixed bug in WHILE loop handling when first statement in macro.

Drivers:

* iMXRT1062: added [E5XMCS_T41](https://www.makerstore.com.au/product/elec-e5xmcst41/) board.

* RP2040: fix to allow ModBus VFDs with BTT SKR Pico board. Issue [#68](https://github.com/grblHAL/RP2040/issues/68).

---

<a name="20230718"/>20230718

Core:

* Some tweaks for new Web Builder options++
* Fixed regression in VFS file system handling causing hardfault when only one mount is present.

Plugins:

* Fans: Updated for core change.

* WebUI: Updated for core setting handling changes.

---

<a name="20230714"/>20230714

Core:

* Added support for traversing directory structure across file system mounts. Allows access to littlefs mount via ftp.
* Fixed inconsistent \(random\) real-time reporting of cycle start signal by adding a latch to ensure it is reported at least once.

Drivers:

* ESP32: added WiFi settings for country, AP channel and BSSID. Changed default AP password to make it legal, was too short.

* STM32F7xx: added EStop signal handling. Driver now defaults to this for the reset input.

Plugins:

* Networking: improved telnet transmit handling.

* WebUI: added file seek function for embedded files, may be used later by gcode macros.

---

<a name="20230711"/>20230711

Core:

* Improved handling of critical events for adding `|$C=1` to full realtime report.

Drivers:

* STM32F1xx: fix for [ioSender issue #312](https://github.com/terjeio/ioSender/issues/312), EStop event not triggered.

Plugins:

* Networking: fix for YModem upload to SD card failure via Telnet or WebSocket connection.

---

<a name="20230708"/>20230708

Drivers:

* STM32F4xx: added four lane SDIO SD card support \(for BTT SKR 2.0 - only tested with a NUCLEO-F446ZE dev board\). [Issue #123](https://github.com/grblHAL/STM32F4xx/issues/123).

* ESP32 and RP2040: fixed "bug" in $I `NEWOPT` string, "FTP" was added even if the FTP protocol was not active.

Plugins:

* SD card: added `.cnc` and `.ncc` file types to default filter.

---

<a name="20230704"/>Build 20230704

Core:

* Skip calling tool change code if current and selected tool number are equal.
* Added 5s timeout/abort handling when waiting for index pulses prior to starting spindle synchronized motion.
* Added definitions for M401 (deploy probe) and M402 (stow probe) for plugin use.
* Added core support for probe protected message and alarm. Requires driver support for interrupt handled probe input.

Drivers:

* STM32F1xx, STM32F4xx and STM32F7xx: simplified and made GPIO interrupt handling more generic/flexible. Fixes [issue #116](https://github.com/grblHAL/STM32F4xx/issues/116).

---

<a name="20230626"/>Build 20230626

Core:

* Added `|$C=1` to full realtime report \(requested by `0x87`\) when controller is in a blocking state \(after a critical event\) but still accepts some `$` commands.

Drivers:

* ESP32: added missing file in _CMakeLists.txt_ and option to enable NGC expression support. Updated Bluetooth code for core changes and added minimum 2ms delay after stepper enable before any motion.

---

<a name="20230610"/>Build 20230610

Core:

* Added virtual Modbus API. Some internal settings handling improvements.

Drivers:

* iMXRT1062, STM32F7xx and RP2040: added Modbus TCP network support.

* STM32F1xx: rerouted _Reset_ signal as _Emergency stop_ per default. Can be overridden in _my_machine.h_ or by setting `COMPATIBILITY_LEVEL` > 1.

* STM32F4xx, ESP32, SAM3X8E and MSP432P401R: minor changes to handle new location of Modbus API.

Plugins:

* Spindle: refactored and renamed Modbus RTU code as a driver implementation for the core Modbus API.

* Networking: added Modbus TCP driver for core Modbus API with support for up to 8 devices. Default is four.  
Added WIZNet support for Modbus TCP.  
Modbus TCP is enabled by bit 2 in the `MODBUS_ENABLE` symbol in _my_machine.h_: `#define MOBUS_ENABLE 4`. This can be added to the previous define values for enabling Modbus RTU with or without RS 485 direction signal support.  
__NOTE:__ The new core API only supports the Modbus RTU protocol, this will be translated to/from Modbus TCP by the driver implementation.  
User code _can_ bypass the core API and transmit Modbus TCP messages directly if it wants/needs to.  
__NOTE:__ VFD spindle Modbus communication will be routed to Modbus TCP if the VFD device id \(unit id\) matches the Modbus TCP device id.  
For now this is untested and may lock up the controller since the networking stack comes up too late to avoid power up selftest \(POS\) failure.
To be addressed in a later revision if someone with a Modbus TCP capable spindle is willing to test.

* Motors and encoder: updated for core setting handling improvements.

---

<a name="20230607"/>Build 20230607

Core:

* Added initial support for macro based automatic tool changes (ATC).  
Currently macros has to be stored on a SD card or in littlefs and [expression support](https://github.com/grblHAL/core/wiki/Expressions-and-flow-control) has to be enabled.
* Added core events for file system mount/unmount.

Plugins:

* SD Card, macro plugin: implemented automatic hook to tool change functions when tool change macros are found in the root mount directory.  
Tool change macro: _tc.macro_, called on `M6`. \(required\).  
Tool select macro: _ts.macro_, called on `T`. \(optional\).  
__NOTE:__ This functionality needs to be extensively tested by users having access to ATC hardware! [Discuss here](https://github.com/grblHAL/core/discussions/309).

---

<a name="20230606"/>Build 20230606

Core:

* Fixed regression related to CSS \(Constant Surface Speed for lathes\) mode.
* Improved stream handling for native USB streams when another stream claims/releases full control.
Depending on the driver some output, such as real-time reports, will now be sent to the USB stream if it is connected to a client \(detected by DTR signal asserted\).
When a pendant is in control \(via the MPG interface\) the USB interface will no longer block transmission if it is the primary interface and no client is connected.

Drivers:

* iMXRT1061, RP2040, all STM32 drivers, SAM3X8E and LPC176x: added DTR signal state detection and handling for native USB streams.

* STM32F4xx, STM32F7xx: fixed regression in spindle sync code.

* STM32F7xx: added optional SD card lowlevel driver support for the SDIO four lane interface \(in addition to the single lane SPI interface\).
Currently this is running in polling mode, will update to DMA mode later.

---

<a name="20230601"/>Build 20230601

Core:

* Improved experimental support for [program flow control](https://github.com/grblHAL/core/wiki/Expressions-and-flow-control).

Drivers:

* STM32F4xx: added alternative Blackpill map with I2C support and optional spindle sync support. From [issue #121](https://github.com/grblHAL/STM32F4xx/issues/121#issuecomment-1569128257).

* ESP32: Fixed typo in MKS Tinybee 1.0 map.

Plugins:

* SD Card: improved macro support.

---

<a name="20230529"/>Build 20230529

Core:

* Added experimental support for [program flow control](https://github.com/grblHAL/core/discussions/309), mainly available for use in [G65 macros](https://github.com/grblHAL/core/wiki/Additional-G--and-M-codes#codes-available-if-driver-or-plugins-supports-them) stored on SD card or in littlefs.

Drivers:

* STM32F4xx: fix for [issue #116](https://github.com/grblHAL/STM32F4xx/issues/116#issuecomment-1565369604), incorrect spindle RPM reported from encoder.

---

<a name="20230526"/>Build 20230526

Core:

* Expanded ioports API with generalized settings handling++.

Drivers:

* RP2040, iMXRT1062, STM32F4xx: added/updated low-level ioports driver layer for analog output \(PWM\). NOTE: not yet used by any board map files.

* STM32F4xx: workaround for [issue #121](https://github.com/grblHAL/STM32F4xx/issues/121), settings write failure.

* iMXRT1062, STM32F1xx, STM32F4xx, STM32F7xx: Improved frequency range for spindle PWM.

Plugins:

* Networking (WIZnet option): reduced memory footprint, [issue #7](https://github.com/grblHAL/Plugin_networking/issues/7).

---

<a name="20230525"/>Build 20230525

Core:

* Expanded ioports API with some configuration and PWM related functions.

Drivers:

* RP2040: added low-level ioports driver layer for analog output \(PWM\). NOTE: not yet used by any board map files.

---

<a name="20230521"/>20230521

Drivers:

* RP2040: added build support for WIZnet ethernet breakouts \(W5500 and W5100S\).

* iMXRT1062, SAM3X8E, SAMD21: fix for C++ serial code compilation issue.

---

<a name="20230519"/>Build 20230519

Core:

* Extended handling of legacy printable real-time commands due to ESP32 RTOS issue.
* Internal changes to crossbar definitions, improved stream handling and sleep handling.

Plugins:

* Networking: added support for some WIZnet SPI based ethernet breakout boards, currently only W5100S and W5500. Updated for core changes.

Drivers:

* STM32F4xx: added low level driver support for WIZnet SPI based ethernet breakout boards, updated SPI interface for DMA transfer.

* RP2040: added low level driver support for WIZnet SPI based ethernet breakout boards, updated SPI interface for DMA transfer. 
__NOTE:__ Build support for ethernet is not yet ready!

* TI SimpleLink base, TM4C123, MSP432P401R, STM32F4xx, LPC176x, RP2040: added minimum delay from stepper enable to first step pulse.

Web Builder:

* Added options for spindle sync and WIZnet networking for relevant driver and boards combinations.

---

<a name="20230507"/> Build 20230507

Core:

* Added experimental support for [G65 and M99](https://github.com/grblHAL/core/wiki/Additional-G--and-M-codes#codes-available-if-driver-or-plugins-supports-them), call and return from macro.

Drivers:

* ESP32: Fix for [issue #73](https://github.com/grblHAL/ESP32/issues/73) - spindle not disabled on reset/stop.

* RP2040: Added Bluetooth support for Pico W. [Discussion #61](https://github.com/grblHAL/RP2040/discussions/61).

* Some: updated for more flexible Bluetooth configuration.

Plugins:

* SD card and macros: added low-level support for G65.

---

<a name="20230502"/>20230502

Plugins:

WebUI: fixes for [issue #6](https://github.com/grblHAL/Plugin_networking/issues/6) - block disabling of websocket daemon from WebUI v3 and [issue #11](https://github.com/grblHAL/Plugin_WebUI/issues/11) - SSDP failure \(with ESP32 in SoftAP mode\).

---

<a name="20230501"/>Build 20230501

Core: 

* Fixed typos.

* Added `grbl.on_gcode_comment` event - can be subscribed to by plugin code to trap comments and act upon them. Intended for implementation of non-standard functionality outside the scope of gcode and $-commands.

Drivers:

* ESP32: fix for spindle PWM not turning off on stop or soft reset.

---

<a name="20230429"/>Build 20230429

Core:

* Now allows some $-commands while critical events are active or in sleep mode. E.g. settings commands can be used to configure the controller.

* Added work envelope data to [global sys struct](http://svn.io-engineering.com/grblHAL/html/structsystem.html), used by soft limits and jog limit handling. This can be modified by plugin code to restrict motion.

Drivers:

* STM32F1xx: added driver support for ganged and auto-squared axes.

* STM32F4xx: fixed typos.

---

<a name="20230427"/>Build 20230427

Core:

* Fixed some typos, incorrect default value for setting $63 - _Disable laser during hold_ flag. Added VFS property.

Drivers:

* STM32F1xx: fix for [issue #34](https://github.com/grblHAL/STM32F1xx/issues/34), typo blocking GPIO interrupt for pin 4.

* ESP32: improved handling of I2S/GPIO pin assignments, added _CMakeLists.txt_ option to enable custom _my_plugin.c_.

* RP2040: added _CMakeLists.txt_ option to enable custom _my_plugin.c_.

Plugins:

* WebUI: fix for [issue #10](https://github.com/grblHAL/Plugin_WebUI/issues/10) - problem with saving files.

* SDCard: added VFS property.

---

<a name="20230417"/>20230417

Core:

* Added missing files to CMakeLists.txt \(used by RP2040 build\).

Drivers:

* RP2040: Fix for incorrect step signals sent to PIO for ganged axes, [issue #60](https://github.com/grblHAL/RP2040/issues/60).

---

<a name="20230416"/>20230416

Core:

* Added reboot required tag for setting $16 and $38.

Drivers:

* ESP32: Improved handling of mixed I2S and GPIO signalling. Allows fix for [issue #70](https://github.com/grblHAL/ESP32/issues/70).

* STM32F4xx: Improved clock tree handling for the different MCU variants, spindle sync fixes and improvements. Fix for [issue #117](https://github.com/grblHAL/STM32F4xx/issues/117), random pauses.

* RP2040: Fix for incorrect reporting of spindle and coolant states in some configurations. Fixes ioSender [issue #286](https://github.com/terjeio/ioSender/issues/286).

Plugins:

* Embroidery: Fix for incorrect handling of jumps.

---

<a name="20230411"/>Build 20230411

Core:

* Fix for issue #236, dual axis offsets.  
__NOTE:__ handling of negative offset values has changed. The primary motor will now be run to correct for negative offset values by moving away from the limit switch.
Prior to this build the secondary motor was run to move towards the limit switch for negative values.
* Changes to allow use of M4 for laser capable spindles in laser mode even if direction control is not available.

Drivers:

* STM32F4xx: pin mappings fix for [BTT SKR Pro 1.x](https://github.com/grblHAL/STM32F4xx/blob/master/Inc/btt_skr_pro_v1_1_map.h) to avoid IRQ conflicts.

Plugins:

* Networking: improved handling of .gz compressed files by httpd daemon.

* Webui: more fixes for [issue #10](https://github.com/grblHAL/Plugin_WebUI/issues/10), cannot download any file from local FS but preferences.json.

---

<a name="20230409"/>20230409

Plugins:

* Networking: fix for incorrect header returned by httpd daemon for plain .gz files.

* Webui: fixes for [issue #10](https://github.com/grblHAL/Plugin_WebUI/issues/10), cannot download any file from local FS but preferences.json.

---

<a name="20230401"/>Build 20230401

Core:

* Fix for issue #279 - B-axis pin name typo.
* Added HAL capability flags for limit switches supported.
* Improved motor pins preprocessor code.

Drivers:

* ESP32: fix for [issue #64](https://github.com/grblHAL/ESP32/issues/64) and [#67](https://github.com/grblHAL/ESP32/issues/67) - use single input for squaring two axes.

* All: added call for setting new HAL limit switches capability flags.

---

<a name="20230331"/>20230331

Plugins:

* Motors: fix for [issue #11](https://github.com/grblHAL/Plugins_motor/issues/11), potential infinite loop.

---

<a name="20230321"/>Build 20230321

Core:

* Fix for issue #271 - unwanted motion after soft reset when feed hold was active, reported for canned cycle but may occur for other commands as well.
* Added function for properly initializing planner struct, updated core to use it.

Plugins:

* Embroidery: switched to new function for initializing planner struct.

Templates:

* HPGL: switched to new function for initializing planner struct.

---

<a name="20230320"/>Build 20230320

Core:

* Another fix for issue #269 - setting of piecewise spindle linearisation values not working.  

* Fix for incorrect reporting of SD card size.

Drivers:

* ESP32: added sanity checks for http and websocket ports, sets websocket port to http port + 1 if equal.

Plugins:

* Spindle: fix for [issue #18](https://github.com/grblHAL/Plugins_spindle/issues/18) - incorrect spindle RPM limits set from VFD response.

* SD card: fix for incorrect reporting of SD card size.

* Embroidery: added option to use Z limit input as needle trigger.

---

<a name="20230317"/>Build 20230317

Core:

* Fix for issue #269 - setting of piecewise spindle linearisation values not working.

Drivers:

* STM32F3xx, STM32F4xx, STM32F7xx: Updated linker config to allow use of scanf, part of fix for issue #269.

Plugins:

* Keypad: updated for core API changes.

---

<a name="20230316"/>20230316

Core:

* Added preprosessor symbol handling for embroidery plugin.

Drivers:

* iMXRT1062, RP2040, STM32F4xx, STM32F7xx and ESP32: Added option for enabling embroidery plugin.  
__NOTE:__ The plugin requires one interrupt capable auxillary input and SD card support.

Plugins:

* Embroidery: initial commit for testing.

---

<a name="20230312"/>Build 20230312

Core:

* Added event definition for SD card file open, fix for issue #118.

Drivers:

* ESP32: Added aux I/O and cycle start/feed hold inputs to MKS DLC32 board, expanded max aux out to 4.  
__NOTE:__ The aux input 0 port on the MKS DLC32 board does not have an internal pullup.

Plugins:

* SD card: added publish for file open event allowing plugins to take over stream handling. Added `$F+` command for listing all files on card.

---

<a name="20230311"/>20230311

Core:

* Fix for issue #264, stepper motors not disabled when entering sleep mode.  
__NOTE:__ all stepper motors will now be disabled even if the $37 setting is set to keep some enabled.

* Fix for recent regression that disabled G7/G8 handling in lathe mode.

---

<a name="20230302"/>20230302

Core:

* Minor fix for the `grbl.on_state_changed` event - incorrect state published following stop signal when streaming gcode.

Drivers:

* RP2040: Added initial support for I2C display protocol, using DMA transfers. 

* STM32F4xx: Fix for EEPROM issue for ST Morpho CNC board maps.

* STM32F7xx: Fix for missing motor enable outputs for ganged axes and fix for IRQ conflicts for 7+ axis configurations \(reference map\).

Plugins:

* Keypad: I2C display protocol plugin, type fix for homed status field.

---

<a name="20230228"/>20230228

Core:

* Fix for Arduino compiler issue with build 20230227. Added new core event and removed redundant code.

Drivers:

* STM32F4xx: Fixed missing pullup enable for EStop input. __NOTE:__ this may trigger EStop alarm for those who have not wired a switch to this input.

Plugins:

* Keypad: I2C display protocol plugin updated to allow multiple message extensions, simplified code.

---

<a name="20230227"/>20230227

Core:

* Enhanced API, reduced planner RAM footprint. Changed rotary fix implementation.

Drivers:

* RP2040: Fix for [issue #13](https://github.com/grblHAL/RP2040/issues/13#issuecomment-1445217377). Fixed typo and added generic 4-axis board map.

* Most networking capable: fixed incorrect MQTT setting description.

* STM32F4xx: fix for incorrect compiler error beeing reported for boards that polls the Z limit switch input.

Plugins:

* Keypad: I2C display protocol plugin updated to utilize gaps in data packet. Gcode message handling bug fix.

---

<a name="20230221"/>20230221

Core:

* Added definitions for some "standard" I2C API calls, for driver/plugin use.

* Added _grbl.on_gcode_message_ event, can be used by driver/plugin writers to parse the message for extending functionality etc.  
Triggered by `(MSG, "some message")` gcode comments.

Drivers:

* Almost all: updated to match new core defined I2C API. Note that some drivers only has a partial implementation, to be updated on demand.

* ESP32: fixed up W5500 ethernet middle layer driver code, now works in DHCP mode with telnet enabled. Other protocols may work, not extensively tested.  
Note that board map definitions has to be adjusted \(or added\) for ethernet as a SPI port in addtion to an interrupt input is required, none are presently set up for that.

* STM32F4xx: updated some board maps/[Web Builder](http://svn.io-engineering.com:8080/) definitions to allow selection of EEPROM size or disabling EEPROM support altogether.

Plugins:

* Keypad: I2C display plugins updated to probe for display on startup, no longer attaches themself if not present.  
Expanded I2C display interface protocol to add gcode message, when present, by dynamically adjusting I2C message length.

---

<a name="20230220"/>20230220

Plugins:

* Trinamic: Improved handling of per axis homing feedrates.

* Spindle: Fix for issue #255 - fails to compile when single VFD spindle is selected.

* Keypad: Initial commit of I2C display interface protocol. __NOTE:__ this is not yet available for compilation.

---

<a name="20230218"/>Build 20230218

Core:

Fix for Grbl [issue #1823](https://github.com/grbl/grbl/issues/1823) - some full circle arcs beeing ignored.

Drivers:

* STM32F4xx: added optional support for USART6. Fixed preprossor issue.

* STM32F7xx: Fixed preprossor issue.

---

<a name="20230217"/>20230217, update 2.

Core:

* Fixed copy/paste "bug" that caused compilation error if compiling for 6-axes.

Drivers:

* STM32F1xx: updated to support up to 6-axes. Note that ganged/auto squared axes are currently not supported - to be added later.

* STM32F4xx: fixed typos and compile time check for IRQ pin assignments.

* STM32F7xx: fixed compile time check for IRQ pin assignments.

---

20230217

Core:

* Added compile time symbols and names for remaining control inputs, rephrased two alarm messages. No functional changes.

Drivers:

* STM32F1xx: added support for MPG mode and option to use UART5 \(untested\) in RC variants.  
Added support for ioports interface \(aux I/O\) and now allows pin naming for RC variants.  
Changed RAM allocation for Cx variants to accomodate above changes, this might also allow a larger planner buffer size.  
Fixed some inconsistencies in BOARD_MACH3_BOB \(_mach3_bob_map.h_\) - untested.  
__NOTE:__ Cx variants are no longer possible to debug due to limited flash, only the release version can be compiled.

Plugins:

* Motors and plasma: updated for spindle handling refactoring.

---

<a name="20230213"/>Build 20230213

Core:

* First phase of spindle handling refactoring with the aim to support multiple \(up to four\) _simultaneously_ active spindles.
Spindle selection by tool number has also been implemented as an option. More details can be found in the [spindle plugin readme](https://github.com/grblHAL/Plugins_spindle/blob/master/README.md).  
__NOTE:__ This change is quite large, bugs may have sneaked in. Please report any issues encountered ASAP.

Drivers:

* All: Updated for spindle handling refactoring.

* iMXRT1062: updated readme for links to updated/patched libraries for SD card \(uSDFS\) and ethernet. User should switch to the updated/patched libraries if used.

* MSP432P401R: fix for missing file include and serial stream registration.

* RP2040: switched to SDK v1.5.0. Added new settings for RP2040, `$78` for wifi country code and `$337` for AP BSSID of AP to connect to. Both are optional.

* STMF4xx: fix for issue [#110](https://github.com/grblHAL/STM32F4xx/issues/110), incorrect NVIC grouping.

Plugins:

* SD Card: fixed some minor inconsistencies.

* Spindle: updated for spindle handling refactoring. Improved ModBus exception handling in some.  
New spindle select settings and improved functionality, see the [spindle plugin readme](https://github.com/grblHAL/Plugins_spindle/blob/master/README.md) for more.

* Odometer and PPI: updated for spindle handling refactoring.

* Networking: wifi \(ESP32 and RP2040\), changed networking settings from `$30x` range to `$32x` range for station mode.  
Added [MQTT](https://en.wikipedia.org/wiki/MQTT) API for plugin developers, example code using this can be found [here](https://github.com/grblHAL/Templates/tree/master/my_plugin/MQTT%20example).  
__NOTE:__ networking and other plugin settings will be reset on update for ESP32 and RP2040, backup and restore.  

---

<a name="20230129"/>Build 20230129

Core:

* Fixed regression in tool change handling related to refactoring of _config.h_ in build 20230125.

Drivers:

* ESP32: Added all supported VFD spindles as options in CMakeLists.txt, some code cleanup.

---

<a name="20230128"/>Build 20230128

Core:

* Some internal changes to allow homing feed rate override from plugins, reserved setting numbers for per axis homing feedrate settings \(not used by the core\).

Motors plugin:

* Fix for issue [#9](https://github.com/grblHAL/Plugins_motor/issues/9), added settings for per axis homing feed rates.  
__NOTE:__ Per axis homing feedrates will only be used for Trinamic driven axes with sensorless homing enabled, others will still use feedrates as set by the `$24` and `$25` settings.  
__NOTE:__ If more than one axis is homed in a cycle and the homing feedrates differ the cycle will be skipped.  
__NOTE:__ Per axis feedrates is currently for experimental use/testing - may be removed in a later build.

Trinamic plugin:

* Fix for issue [#41](https://github.com/grblHAL/RP2040/issues/41#issuecomment-1379449288), wrong PWM autoscale mode selected for sensorless homing.

---

<a name="20230126"/>Build 20230126

Core:

* Fixed include file issue causing macro warnings for drivers built with the Arduino platform.

Web Builder:

* Added macros plugin to capable drivers, removed resizable planner buffer option as this is now always enabled and added option for rotary axes feed rate fix.

---

<a name="20230125"/>Build 20230125

Core:

* Added settings, updated settings version to 22:  
`$398` - _planner buffer blocks_, default value 35.  
`$481` - auto real time report interval. Default value 0 \(auto reporting disabled\), range 100 - 1000 ms.
When enabled `|AR` is added to the full real time report \(requested by the real time command `0xA7`\).
New real time command `0x8C` can be used to toggle auto reporting on/off.  
`$482` - timezone offset. To be implemented.

* Added parsing of gcode words `O` and `$`. `O` is currently not used by the core, `$` value is for selecting the spindle to address for some gcode commands,
currently only `-1` or `0` is allowed.

* Added _allow null_ flag \(0 for off, 1 for on\) to `$ES` and `$ESH` outputs. When set to 1 for range restricted values they can be set to an empty string or 0 depending on the data type.

* Merged _defaults.h_ with [config.h](./config.h), deleted _defaults.h_.
Changed some symbol/macro names for consistency, e.g. all starting with `DEFAULT_` can be changed at run time with `$` commands. Reorganized file and added doxygen [documentation](http://svn.io-engineering.com/grblHAL/html/config_8h.html).

* Fix for issue [#43](https://github.com/grblHAL/RP2040/issues/43), _Laser mode spindle enable after pause_.

__Note:__ Backup and restore your settings as they will be reset to default values.

Plugins:

* Keypad: added macros plugin, up to 8 macros can be bound to pins and/or to keypad single character commands. Added jog support for A-axis to keypad plugin, extended programming interface.

* Spindle: merged [#14](https://github.com/grblHAL/Plugins_spindle/pull/14) - fix typo on GS20 driver to set RPM correctly.

* Fans: added setting `$483` for linking spindle enable signal to fan\(s\) enable.

Drivers:

* All: updated for settings version change.

* ESP32: added board map for OpenBuilds BlackBox X32, fix for issue [#48](https://github.com/grblHAL/ESP32/issues/48), [#49](https://github.com/grblHAL/ESP32/issues/49), [#50](https://github.com/grblHAL/ESP32/issues/50), [#56](https://github.com/grblHAL/ESP32/issues/56) \(needs end user testing\) and [#58](https://github.com/grblHAL/ESP32/issues/58) plus some board map updates/fixes.

* iMRX1062: now maps spindle enable and spindle direction signals as aux outputs if a single VFD spindle is configured.

* RP2040: fix for motor plugin issue [#7](https://github.com/grblHAL/Plugins_motor/issues/7#issuecomment-1381331011).

* SAM3X8E: added probe pin assignment for Ramps 1.6 board.

* STM32F1xx: some bug fixes, added board map for MACH3 breakout board.

* STM32F4xx: fixed IRQ conflict for BTT SKR v1 and v2 boards by reducing max number of allowed auto squared axes.

* STM32F7xx: some bug fixes, merged PR [#11](https://github.com/grblHAL/STM32F7xx/pull/11) and [#12](https://github.com/grblHAL/STM32F7xx/pull/12).

---

<a name="20221101"/>Build 20221101

Core:

* Added single axis homing commands for U and V, remapping of ABC homing commands to UVW when configured.

Plugins:

* WebUI and Networking: fixes for compiler warnings.

Drivers:

* ESP32: fix for Web Builder failing when networking was enabled.

---

<a name="20221115"/>Build 20221115

Core:

* Updated settings reports to output correct units and descriptions for settings related to axes configured as rotary.  
__Note:__ Senders may have to be restarted to display these after a configuration change \($376 - rotatational axes\) and even then they might not display them correctly.

* Internal change to allow flagging settings to have a minimum value or length different from 0 and still allow to them to be 0 or have length 0.

Plugins:

* Some: updated for core settings change.

Drivers:

* Some: updated for core settings change.

---

<a name="20221101"/>Build 20221101

Core:

* Added single axis homing commands for U and V, remapping of ABC homing commands to UVW when configured.

Plugins

* WebUI and Networking: fixes for compiler warnings.

Drivers:

* ESP32: fix for Web Builder failing when networking was enabled.

---

<a name="20221031"/>20221031

Core:

* Minor change to suppress compiler warnings for SAM3X8E (Arduino Due - bad framework code).

Drivers:

* SAM3X8E : More compiler warnings supression, added missing PWM output for Mega2560 board map.

* Some: Updated Web Builder definitions.

---

<a name="20221028"/>Build 20221028

Core:

* Minor bug fix, preprocessor tuning.

Drivers:

* LPC176x: fix for issue [#31](https://github.com/grblHAL/LPC176x/issues/31).

* Some: Fixed some obscure bugs, updates for Web Builder.

---

<a name="20221023"/>Build 20221023

Core:

* Fix for issue #209, incorrect handling of homing of more than one auto squared axis in each pass. Error 55 will now be returned.

---

<a name="20221022"/>Build 20221022

Core:

* Fix for old regression, issue #204.

* Now hides spindle PWM related settings if no PWM spindle is available.

Drivers:

* Many: Fixes for incorrect code related to new spindle type property introduced in build 20221018. Updated [Web Builder](http://svn.io-engineering.com:8080/) data.

---

<a name="20221018"/>Build 20221018

Core:

* Added new setting `$346` for action to take after tool change: either return controlled point \(tool tip\) back to the same position as before the M6 command \(default\) or move spindle to Z home only.

* Added spindle type property to HAL, "hardened" code.

* Fix for issue #191, allow homing of rotary axes with infinite rotation \(max travel = 0\).

Plugins:

* Spindle: Added spindle type property to registration data.

* Fans: Bug fix, added off delay option for fan 0 with setting `$480` specifying number of minutes to delay. Useful for allowing an exhaust fan to clear a laser cutter enclosure before turning it off.  
__Note:__ The new setting may cause a reset of other plugin settings to default values, backup and restore.

* Laser coolant: implemented off delay and coolant lost monitoring.

Drivers:

* STM32F4xx: Fix for [issue #99](https://github.com/grblHAL/STM32F4xx/issues/99). Updated spindle registration.

* SAM3X8E: Fix for [issue #15](https://github.com/grblHAL/SAM3X8E/issues/15).

* Many: Improved driver PWM spindle code.

* Some: Updated for [Web Builder](http://svn.io-engineering.com:8080/) requirements.

---

<a name="20221009"/>Build 20221009

Core:

* Fix for bug preventing some hosts \(Win7\) querying SSDP information.

Plugins:

* WebUI: Removed ESP32 dependency in WiFi SoftAP mode.

Drivers:

* ESP32: Updated for WebUI change.

* RP2040: Fixed WiFi AP mode [issues](https://github.com/grblHAL/RP2040/issues/34), fixed bug in `$314` setting reporting.  
__NOTE:__ AP mode IP address cannot be changed from the default `192.168.4.1` address.  
__NOTE:__ There are limitations/bugs in the SDK preventing SSDP queries from arriving, this is likely to be fixed in a later SDK release.

---

<a name="20221005"/>Build 20221005

Core:

* Added optional string pointers to HAL for driver and board URLs. If present they are announced by the SSDP protocol.

Plugins:

* Networking: Added mDNS and SSDP protocol support.  
__Note:__ Some drivers require manual patching before enabling.

* WebUI: Fix for bugs affecting settings handling. [Issue #5](https://github.com/grblHAL/Plugin_WebUI/issues/5) and [issue #6](https://github.com/grblHAL/Plugin_WebUI/issues/6).  
__Note:__ Issue 6 was about incorrect handling spaces in string settings, these are not permitted in hostnames according to [RFC1123](https://www.rfc-editor.org/rfc/rfc1123) and I may add validation later.

Drivers:

* iMXRT1062, RP2040, ESP32, STM32F7xx and MSP432E401Y: Added options for enabling mDNS and/or SSDP protocols.  
__Note:__ iMXRT1062 and RP2040 require manual patching before enabling.

* STM32F1xx: Added two alternatives for spindle PWM pin assignment.

* STM32F4xx: Added support for PWM output on PE5 and PE6 \(for BTT SKR 2.0\).

* STM32F7xx: Updated for latest STM32CubeIDE device driver HAL.

Templates:

* my_plugin/hpgl: Added tentative support for most HP7475A `ESC . ...` device control sequences. _Testing required!_

---

<a name="20220928"/>Build 20220928:

Core:

* Added crossbar support for UART RTS handshake output signal.

Drivers:

* iMXRT1062: Fix for issue [#48](https://github.com/grblHAL/iMXRT1062/issues/48), coolant and mist outputs swapped.

* SAM3X8E: Added SMART Ramps board map. By @MrAntza99.

* RP2040: Added option to use RTS handshaking for UART comms via the primary port. Updated the _citoh_cx6000_ map to use it if UART comms is enabled.

* ESP32: Added laser plugin submodule and updated CMakeLists.txt with enable options for it.

Templates:

* my_plugin/hpgl: Added trapping of more HP7475A \(possibly all?\) `ESC . ...` device control sequences to avoid errors/hangs.

---

<a name="20220925"/>Build 20220925:

Core:

* Added `[AXS:<number of axes>:<axisletters>]` line to `$I` report response, replaces the string `ABC2UVW` from the `NEWOPT` element in the `$I` response.  

* Fixed `|Pn:` real time report element pin state conflict: `F` is now used for motor fault and `M` for motor warning, `U`, `V` and `W` for limit switch status.

Further details can be found in the [wiki](https://github.com/grblHAL/core/wiki/Report-extensions).

Plugins:

* Laser: Added experimental support for LaserBurn clusters, for faster engraving.

* SDCard: Minor tweak to enable plugins to modify the file stream without losing real time report extensions.

Drivers:

* ESP32: added directory for embedded read-only files and moved related files there.

---

<a name="20220922"/>Build 20220922:

Core:

* Changed signature of [spindle_update_caps()](http://svn.io-engineering.com/grblHAL/html/spindle__control_8h.html#a3170b0136a49e0b30047e00bdf4e812c), third party developers must update _driver.c_ if used.

* Removed some superfluous code, improved handling of laser mode M4 for jogging and motion complete event.  
Bug fixes + expanded step/dir map to 8 axes.

Drivers:

* All: Updated for core signature change.

* STM32F1xx: updated FatFs link to v0.14.

---

<a name="20220920"/>20220920:

Plugins:

* Networking: Fix for http daemon not decoding the base URL when no URL handlers were registered.

* WebUI: Fix for incorrect parsing of ESP701 action parameters.

* Laser (PPI): Bug fixes.

---

<a name="20220918"/>Build 20220918:

Core:

* Added [configuration option](https://github.com/grblHAL/core/blob/master/config.h) `BLOCK_BUFFER_DYNAMIC` for dynamically allocate planner buffer. If enabled setting `$398` can be used to set number of blocks in the buffer.  
__NOTE:__ A restart of the controller is required after changing `$398`.  
__NOTE:__ If not enough free memory is available the actual allocation size will be reduced until it fits. The actual allocation can be checked with `$I`.  
Each block requires around 100 bytes of memory.  
__NOTE:__ All setting values will be reset when this option is changed, backup and restore!  
__NOTE:__ In a later version this option will be removed and dynamic allocation will become standard.  

* Added experimental [configuration option](https://github.com/grblHAL/core/blob/master/config.h) `AXIS_REMAP_ABC2UVW` for remapping ABC axis letters to UVW.  

Drivers:

* ESP32: Applied workaround for wifi/pin36/pin39 silicon bug. Reenabled HAL function for reboot.

* STM32F7xx: Added support for up to 8 axes in the driver, with the reference board map only. As of now untested but it compiles and runs!  
Are there any senders that can candle 8 axes available? [ioSender](https://github.com/terjeio/ioSender) can not but that may change.

Plugins:

* WebUI: added new and missing settings options for ESP400, added axisletters from configuration to ESP800 response.

* SDCard: updated FatFs VFS wrapper for read-only configuration.

* Some: moved reboot required message for some settings to reporting by using a settings flag.

---

Build 20220916:

Core:

* Added setting definitions for WebUI client inactivity timeout and real time report auto interval.

Plugins:

* WebUI: Added support for multiple client switchover and session inactivity timeout. Updated for websocket API changes.  
Reorganized ESP v2 and v3 protocol code for readability and added settings for client inactivity \(`$396`\) real time report auto interval \(`$397`\).  
__NOTE:__ This will reset WebUI settings to default and possibly other plugin settings too. Backup and restore!

* Networking: Enhanced websocket daemon API, now allows multiple clients - with the limitation that only one can claim the websocket "serial" stream.

---

20220914:

Core:

* Added optional HAL entry point for getting free memory \(ideally sum of free blocks on the heap\), currently used by WebUI.

Plugins:

* WebUI: Sort ESP400 output by settings group, fixed ESP410 response and added free memory to ESP420 response when available from the HAL.
Updated for websocket API changes.

* Networking: Removed WebUI specific code from websocket daemon, made API more flexible. Some bug fixes and a bit of code cleanup/refactoring.

* SDCard: Switched to VFS file handling for the YModem protocol.

Drivers:

* iMXRT1062, RP2040, ESP32 and STM32F7xx: added support for free memory HAL entry point.

* ESP32: improved wifi AP scanning.

* MSP432E401Y: added littlefs and WebDAV support.

---

20220912:

Plugins:

* WebUI: Improved ESP800 response, updated embedded fallback page and fixes for file listing error and AP mode handling.

---

20220911:

Core:

* Minor options preprocessor change for littlefs, "hardened" VFS code. Core build date not changed.

Plugins:

* WebUI: Added _all_ grblHAL settings to ESP400 response, file systems handling improvements++. Still WIP, but getting closer to final version!

* Networking: some general enhancements \(for WebUI\).

* SDCard: added VFS wrapper for littlefs. This plugin will be renamed later, likely to _Plugin_storage_.

Drivers:

* iMXRT1062, RP2040 and ESP32: Added [littlefs](https://github.com/littlefs-project/littlefs) support for flash based file storage.

* STM32F7xx: Fixed [typo](https://github.com/grblHAL/core/issues/186#issuecomment-1242943739).

---

20220906:

Core:

* Added `$RTC` system command for outputting or setting current real time clock date and time. Uses ISO8601 format.  
__Driver developers:__  
_grbl/limits.h_ has been renamed to _grbl/machine_limits.h_ (along with the _.c_ counterpart).  
[hal.enumerate_pins](http://svn.io-engineering.com/grblHAL/html/structgrbl__hal__t.html#a661c9aa458a2e6fc5fb1657e121999a3) and the associated [callback function](http://svn.io-engineering.com/grblHAL/html/hal_8h.html#a41e902cfc3da615f9494aba956d895ba) parameter has a new signature, a void pointer has been added. Driver implementations should pass this on to the callback.  
The HAL version number has been increased to 10 due to this, update _driver.c_ to match!

Plugins:

* WebUI: added many ESP commands to v3 command handler, some code refactoring. Still WIP.

* Networking: some minor bug fixes.

Drivers:

* All: updated for grbl/limits.h name change and HAL version number increase.

* iMXRT1062, STM32F4xx, STM32F7xx and ESP32: Added RTC support.

* ESP32: WebUI backend support improved.

---

20220904:

Core:

* Added optional RTC \(Real Time Clock\) support to the HAL. VFS improvements.

Plugins:

* Networking: improved websocket subprotocol handling. 

* WebUI: separated command handlers for v2 and v3 and improved detection of v3 clients. Now sets RTC from ESP800 if HAL allows.

Drivers:

* RP2040: Added RTC support++.

* iMXRT1062: updated uSDFS patch - needed for VFS changes.

* STM32F7xx: updated FatFs options file for better VFS integration.

---

20220903:

Core:

* Tuning, VFS improvements.

Plugins:

* Networking: improved timestamp handling, some other minor fixes. 

* WebUI: reverted to stream writer for command output \(for now\). Added RP2040 \(Pi Pico W\) to supported boards.

Drivers:

* RP2040: Added networking and WebUI support for Pico W.  
__Note:__ Soft AP mode is WIP and not fully functional.  
__Note:__ NVS storage of settings has been moved to the end of flash, backup and restore!

* STM32F7xx: committed missing update of .cproject file.

---

20220825:

Core:

* Added virtual file system \(VFS\) handler, Linux/Unix style with mount directories.
* Now raises alarm if homed state becomes invalid on settings changes when homing on startup is required. Issue #173.

Plugins:

* Networking: added optional \(and initial\) support for WebDAV protocol to http daemon.  
Added virtual file systems \(VFS\) for temporary RAM storage, direct or via `hal.stream.write` output.  
__Note:__ Saving files with WebDAV for Windows mounts does not work, some weird things going on like initial save beeing for a zero sized file.  
Tested ok with WinSCP.

* WebUI: Switched to use virtual file system \(VFS\) for file handling.

* SDCard: Swithed to use virtual file system \(VFS\) for file handling. Added VFS implementation for FatFS, mounted as root \(/\).

* Spindle: Added GS20 and YL620 VDF spindles from [PR#9](https://github.com/grblHAL/Plugins_spindle/pull/9) by @andrewmarles.  
Added option for extending VFD spindle functionality generically. Potential fix for core [issue #177}(https://github.com/grblHAL/core/issues/177).

* Encoder: fixed settings registration bug, [issue #1](https://github.com/grblHAL/Plugin_encoder/issues/1).

Drivers:

* ESP32: Switched to grblHAL http daemon and full use of WebUI plugin. Added virtual file system mounts \(VFS\) for SPIFFS and embedded files.

* iMXRT1062: additional fix for encoder plugin [issue #1](https://github.com/grblHAL/Plugin_encoder/issues/1).

* ESP32, iMXRT1062, STM32F7xx:* ESP32, iMXRT1062, STM32F7xx: Added configuration option for WebDAV protocol to _my_machine.h_.

---

20220801 \(2\):

Core:

* Added masking of password settings values for WebUI clients.

Plugins:

* Networking: more include file fixes.  

* WebUI: Fixed incorrect type mapping for password settings.

Drivers:

* ESP32, STM32F7xx, iMXRT1062: added actual MCU frequency to new HAL struct field \(used by WebUI for reporting system information\).  

---

20220801:

Core:

* Added MCU frequency variable to the HAL struct - to be set by the various drivers later on. 

Plugins:

* Networking: fixed some include files dependencies that caused issues in some configurations.  

* WebUI: added missing guards for SD card enabled and new compile time option. Added missing status message to file listings.

Drivers:

* ESP32: added missing guards that caused compilation errors if WebUI option was enabled without SD card enabled and/or authentication enabled.  
Added option for auto push the real time status report.

---


20220731:

Core:

* Added core event `on_homing_completed`, some sanity checks on MPG stream registration.

Drivers:

* ESP32: Added IP address to "WIFI STA ACTIVE" message published on startup when IP address has been assigned.

* STM32F7xx: Fixed _Release_ build settings, added _.bin_ output.

---

20220729:

Core:

* Updated for issue #169 - extend range of axis auto square offsets.

Plugins:

* WebUI: moved SD card handling to separate file for sharing with the ESP32 driver. Still WIP.

Drivers:

* ESP32: Switched to plugin code for WebUI login and SD card handling. Still WIP.  
__NOTE:__ Settings for WebUI passwords has been moved to the WebUI plugin, this will trigger a reset of the network settings!

* SAM3X8E: Fix for issue [#11](https://github.com/grblHAL/SAM3X8E/issues/11).

---

20220728:

Plugins:

* WebUI: "Hardened" code, a bit of cleanup. Still WIP.

Drivers:

* ESP32: Attempted fix for TMC2209 UART comms, [ESP32 issue #33](https://github.com/grblHAL/ESP32/issues/33). Still not 100% reliable.

---

20220724:

Core:

* Added authentication related error codes and messages. Added reboot required per setting to various reports.

Plugins:

* Networking: Various fixes/enhancements for WebUI 3 support.

* WebUI: Added support for forms based authentication++ Still WIP.

Drivers:

* ESP32: Some minor bug fixes, added TMC2209 option to CMakeLists.txt \(untested\).

* STM32F4xx: Fix for [issue #83](https://github.com/grblHAL/STM32F4xx/issues/83).

* STM32F7xx: Fixed regression introduced in previous build.

---

20220722:

Core:

* Added settings flag for reboot required \(currently unused\) and API function for getting setting group details.  

Plugins:

* WebUI: refactored ESP command handling, added some commands. Added [preliminary build](https://github.com/grblHAL/Plugin_WebUI) of WebUI v3 for testing. Still WIP.

Drivers:

* ESP32: Added board map for MKS Tinybee v1, improved I2S pin handling and secondary serial port options.

* STM32F4xx: Fix for [issue #82](https://github.com/grblHAL/STM32F4xx/issues/82).

* STM32F7xx: Added handling of Trinamic driver pins \(currently unused - no Trinamic support yet\).

---

20220720:

Plugins:

* WebUI: fixed ESP701 response.

Drivers:

* SAM3X8E: added tentative support for MPG mode.

---

20220718:

Core:

* Added support for `G5.1` \(quadratic spline\) and multi turn arc option for `G2` and `G3` via P-word.
* Some additions to data structures used by plugins.

Plugins:

* Networking: added optional function to API for querying current status of connection and running services. Required for the WebUI plugin.  
Added support for client side ping message used by WebUI v3.
* WebUI: added initial support for [WebUI v3](https://github.com/luc-github/ESP3D-WEBUI/discussions/94#discussioncomment-2861616) messaging.
* SDCard: added function call for querying current job status.

Drivers:

* iMXRT1062, STM32F7xx, ESP32, TM4C1294 and MSP432E401Y: Implemented optional API function for querying current status of connection and running services.

---

20220710:

Core:

* Moved initial stepper enable call to the core (from drivers).

Drivers:

* Most: Updated for move of stepper enable initial stepper enable call to the core.

* STM32F4xx: Fix for missing code guard, updated my_machine.h options.

* STM32F7xx: Fix for incorrect `$pins` report for SPI pins.

---

20220709:

Core:

* Fix for incorrect sequencing of init calls when corexy and backlash compensation is enabled at the same time, [ESP32 issue #25](https://github.com/grblHAL/ESP32/issues/25).

* Added call to driver to immediately set stepper enable signals when `$37` \(Stepper deenergize\) is changed.

* Some minor improvements in settings handling and options reporting.

Drivers:

* iMXRT1062: Updated [SD card driver patch](https://github.com/grblHAL/iMXRT1062/tree/master/patches) with workaround for non word-aligned writes that caused file corruption. Fix for [WebUI issue #4](https://github.com/grblHAL/Plugin_WebUI/issues/4).

* STM32F4xx: Added option for using timer 2 for spindle sync RPM timer as this allows spindle sync for low pin count MCUs.  
Note that timer 2 is a 16 bit timer that had to be extended virtually to 32 bit - this _may_ affect spindle sync operation/performance.

Templates:

* Added plugin for Marlin style M17/M18 (M84) commands for enabling/disabling stepper drivers as fix for issue #184. 

---

20220703:

Core:

* Deprecated setting `$7` \(can be set as before but is no longer reported\). Added setting `$9` for PWM spindle options:
```
$9: PWM Spindle as bitfield where setting bit 0 enables the rest:
    0 - Enable (1)
    1 - RPM controls spindle enable signal (2)
```
Bit 1 in this setting replaces setting `$7`, bit 0 controls the PWM output.  
__NOTE:__ M3 and M4 with S0 will now set the spindle enable output if `$9` is `1`. Ref [issue #156](https://github.com/grblHAL/core/issues/156).  
__NOTE:__ the change is not backwards compatible with current 3rd party drivers, these has to be updated to match changes in the core.


Drivers:

* All: Updated for core changes related to the new `$9` setting.

* ESP32: Prepared driver for ethernet interface.

---

20220625:

Core:

* Reverted config.h change that enabled COREXY as default. ESP32 issue [#31](https://github.com/grblHAL/ESP32/issues/31).

Drivers:

* ESP32: Updated CMakeLists.txt for spindle plugin changes.

Plugins:

* Spindle plugin: Added H100 VFD driver \(untested\).

---

20220618:

Core:

* Added grbl.on_reset event, new settings for VFDs++

Plugins:

* Spindle plugin: Added VFD manager, simplified VFD registration \(no core changes required when adding a new type\), added MODVFD VFD support, added `M104Q<n>` M-code for selecting spindle.  
Some of these changes were adopted from [PR#9](https://github.com/grblHAL/Plugins_spindle/pull/9).

---

20220616:

Core:

* Delayed calling `hal.driver_reset` until alarm and abort states has been established.

Drivers:

* STM32F4xx : Fix for [issue #77](https://github.com/grblHAL/STM32F4xx/issues/77) - serial port clock selection.

Plugins:

* Spindle plugin: Fix for [issue #9](https://github.com/grblHAL/plugins/issues/9) - VFD spindle not stopped on STOP command.

---

20220612:

Core:

* Changed kinematics API and implementations (corexy and wallplotter) to allow backlash compensation. Ref [ESP32 issue 25](https://github.com/grblHAL/ESP32/issues/25).

* Fixed feed rate handling for corexy kinematics. Ref issue #147.

* Fixed tool table/tool change bugs. Ref. [ioSender issue 228](https://github.com/terjeio/ioSender/issues/228).

Drivers:

* iMXRT1062 : Fix for issue #38.

* STM32F4xx : Added missing code guard for boards not having a spindle direction pin. Improved UART channel assignment handling.

* STM32F7xx : Improved UART channel assignment handling.

* ESP32 : Added missing file \(corexy.c\) to filelist, fixed incorrect URL in readme.

---

20220416:

Drivers:

* SAM3X8E: Fix for [issue #7](https://github.com/grblHAL/SAM3X8E/issues/7).

* LPC176x: Allowed auto squared axes for RAMPS 1.6 board, routed serial port to the WiFi port when USB serial is disabled for BTT SKR v1.4 Turbo board.

* STM32F4xx: Fix for [issue #69](https://github.com/grblHAL/STM32F4xx/issues/69). Added PB9 as option for spindle PWM.

* RP2040: Added support for spindle plugin, [issue #26](https://github.com/grblHAL/RP2040/issues/26).

---

20220325:

Core:

* Changed spindle handling to allow dynamic spindle registration and support for multiple spindles selectable at run-time.  
If more than one spindle is to be made available [grbl/config.h](https://github.com/grblHAL/core/blob/master/config.h) has to be edited and the symbol `N_SPINDLE` has to be increased from the default value of 1 to the number of spindles to allow \(currently max 8\).  
When more than one spindle is registered setting `$395` becomes available for specifying the spindle to enable at startup. Use `$$=395` to output a list of available spindles and the corresponding spindle id to use for configuration.  
__NOTE:__ Using `$32` for switching between a PWM spindle and a VFD spindle is no longer supported, either use `$395` or `M104P0` to select the PWM spindle, `M104P1` to select the configured VFD spindle.
Note that laser mode \(`$32=1`\) will be disabled if the active spindle is not a PWM spindle.   
__NOTE:__ the change is not backwards compatible with current 3rd party drivers, these has to be updated to match the changed core.

Plugins:

* The PPI and spindle plugins has been updated for the new spindle handling. 

Drivers:

* All: updated for the new spindle handling.  
__NOTE:__ I have only done limited testing of the changes, please report any problems!

---

20220315:

Drivers:

* LPC176x: Fix for issue #122, polling of limit switches during homing.

* SAM3X8E: Fix for issue #124, limit signals inversion did not work.

* RP2040: [Fixed typo](https://github.com/grblHAL/RP2040/issues/16#issuecomment-1058560536) in enable/disable of secondary UART port.  
Fixed incorrect check for UART TX shift register empty and handling of stepper enable when Trinamic drivers are enabled.  
Updated BTT SKR Pico map - may still need adjustments...

---

Build 20220215:

Core:

* Added a MPG mode option for using input pin for mode switching when enabled together with keypad plugin. Added pin descriptions for UART pins used for MPG.

Plugins:

* Keypad: added block for MPG mode change real-time command when input pin is used for mode switching.

Drivers:

* Some: updated for new MPG mode option.

* ESP32: bug fix for random guru wakeups and subsequent crash for I2S enabled boards.

---

Build 20220210:

Core:

* Fixes for backlash compensation: no longer resets current direction on a soft reset/stop, added handling of backlash setting changes per axis.

---

20220209:

Plugins:

* Trinamic: "Bug fix" for soft UART TMC2209 drivers.

Drivers:

* LPC176x and STM32F4xx: Refactored soft UART code used for TMC2209 drivers. Hopefully this fixes remaining issues, however I do not have the actual hardware to verify the code.

---

Build 20220204:

Core:

* Fix for regression where laser mode / lathe mode settings were ignored.

Plugins:

* Motors: Added 100ms delay before attempting to initialize Trinamic drivers after power up. At least TMC2130 require this.

Drivers:

* STM32F4xx: Removed Trinamc driver restriction for ganged axes configs for BTT SKR 2.0 board. Corrected linker script used for Nucleo F411 builds.

---

20220203:

Plugins:

* Networking: Added critical sections for FreeRTOS builds and bit of code harmonization for the telnet and websocket daemons.

Drivers:

* ESP32: Increased FreeRTOS tick rate to 1KHz, pinned tcp task to core 0 and increased network daemons polling rate. Hopefully this along with networking plugin changes fixes issue [#10](https://github.com/grblHAL/ESP32/issues/10).
* iMXRT1062: Added workaround for potential compiler bug, added small delay to PJRC USB polling to avoid overwhelming senders with responses in _Check mode_. This should fix issue [#31](https://github.com/grblHAL/iMXRT1062/issues/31).

---

Build 20220131:

Core:

* Fixed tool change and spindle stop on reset/stop commands regressions.

Drivers:

* ESP32: Updated WiFi stream handling on network connect/disconnect to use shared code in the core.

Plugins:

* WebUI: Added number of configured axes to firmware status report. Issue [#18}(https://github.com/grblHAL/ESP32/issues/18).
* Networking: Added close \(stream\) connections calls.

---

Build 20220123:

Core:

* The symbol `ENABLE_SAFETY_DOOR_INPUT_PIN` in _grbl/config.h_ has been replaced with `SAFETY_DOOR_ENABLE` in _my_machine.h_.
* Changed probe touch off handling to reduce deceleration overshoot.  
Uncomment `//#define MINIMIZE_PROBE_OVERSHOOT` in _grbl/stepper.c_ to enable this feature.  
Later it may be permanently enabled if no side-effects are experienced, test with care!

Drivers:

* Most: Added `\\#define SAFETY_DOOR_ENABLE 1` to options in _my_machine.h_, uncomment to enable. Requires board support for safety door input.
* RP2040: Added pin map and support code for the [C.ITOH CX-6000 plotter project](https://hackaday.io/project/183600-citoh-cx-6000-plotter-upgrade). Uses the HPGL and keypad plugins.
* RP2040: Added tentative support for BTT SKR Pico 1.0 board. Incomplete and untested!
* LPC176x: Made TMC2209 UART driver support for BTT SKR E3 Turbo board available for BTT SKR V1.4 Turbo, still untested!

Templates:

* Added [HPGL template plugin](https://github.com/grblHAL/Templates/tree/master/my_plugin/hpgl). Replaces the G-code parser with a HPGL parser and is a good example of grblHAL programmability. _Not a single line of code was changed in the core for this!_

---

Build 20220111:

Core:

Added new core event triggered during looping when executing a millisecond delay.
Added `ISR_FUNC` macro for placing time critical functions in RAM.

Drivers:

* RP2040: Limited max time between step pulses to avoid jog movements taking too long to complete. Moved time critical code run in interrupt context to RAM.
* iMXRT1062: Fixed memory leak in ioports code.
* Many: Forced ioports numbers \(_Aux \<n\>_\) to be contiguous regardless of how they are defined in the map file. 

Plugins:

* Spindle \(Modbus\): Added subscription to new core event to poll for responses during delays, fixes issue with spindle at speed check for some drivers.
* Some: Moved time critical code run in interrupt context to RAM (for RP2040 driver).

---

Build 20220109:

Core:

* Added optional spindle select event to core events, for dual spindle enabled configurations \(VFD + PWM spindle\).
* Attempted fix for weird issue with VFD spindle when spindle at speed tolerance is set > 0 with `$340`.

Plugins:

* Networking: fix for [dependency issue](https://github.com/grblHAL/core/discussions/106) with iMXRT1062 driver when lwIP library was not installed.
* Spindle: updated Huanyang VFD driver to support the new spindle select event.  
Added experimental M-code for switching spindles, `M104P0` for PWM spindle, `M104P1` for VFD spindle. Only available in `DUAL_SPINDLE` configurations.
* Keypad: added many new single character commands, mostly the same as available as standard real time commands.

Drivers:
* STM32F4xx: fixed name case for FatFs driver directory, added FatFs R0.13c source.
* iMXRT1062, STM32F4xx and SAM3X8E: added `DUAL_SPINDLE` configuration option \(for testing\).

---

Build 20220105:

Core:

* Moved and enhanced handling of pendant serial stream \(MPG\) to core. Added some new stream handling functions.
* Added real time command character `0x8B` for toggling MPG stream on/off.
* Fix for issue #104.

Plugins:

* Bluetooth: updated to use new stream open function.
* Keypad: added MPG stream toggle support, bug fix.
* Networking: refactored and renamed telnet and websocket daemons.
* Spindle: changed to get actual RPM from encoder when available.

Drivers:

* iMXRT1062, ESP32, STM32F7xx, MSP432E401Y and TM4C1294: updated for networking plugin refactor.
* iMXRT1062, RP2040, ESP32, STM32F4xx, STM32F7xx, MSP432E401Y and TM4C1294: updated for pendant serial stream handling changes.  

---

Build 20211226:

Core:

* Fix for spindle sync PID $-settings not showing up when spindle sync option is enabled.
* Made `$392` \(spindle spin up delay\) available when spindle driver supports spindle at speed functionality.

Drivers:

* iMRX1062, STM32F4xx, STM32F7xx and MSP432: Fix for changes to spindle sync PID $-settings requiring a hard reset to take effect.

---

Build 20211223:

Core:

* Added function for connecting via specified serial port.
* Changed internals to allow further VFD spindle drivers to be added.

Plugins:

* Bluetooth and Modbus: increased number of serial ports that can be selected from \(up to 4 - driver dependent\).

Drivers:

* Many: updated for changed VFD internals and shared I2C symbols.
* ESP32: "hardened" code.
* LPC176x: fixed bug in handling of counterclockwise \(CCW\) spindle direction.
* STM32F4xx: added support for selecting primary serial port driver in board map specific code.

---

Build 20211218:

Core:

* Fixed issue where VFD spindle alarm during homing failed to stop movement. Changed homing init to only stop spindle and coolant if already on.

Plugins:

* Trinamic: Bug fixes and improvements for ganged axes handling.

Drivers:

* All STM32 drivers, LPC176x, MSP432E401Y and TM4C1294:  
Made folder references relative in Eclipse .cproject file to allow renaming of project.
* STM32F4xx:    
Added alternative startup code for F407 and F446 to allow use of additional peripherals in user code.  
__NOTE:__ This may break PlatformIO compilation. A possible workaround is to delete the startup folders not matching the MCU variant.  
Added support for Bigtreetech SKR 1.2 boards \(as a synonomym for SKR 1.1\), switched soft UART pins for Trinamic drivers to direct connection.
* ESP32: Bug fixes for ganged axes and output pins with pin number > 31.
* SAM3X8E and SAMD21: Simplified USB polling.

---

Build 20211213:

Core:

* Added generic stream switcher functions for driver use, to avoid duplicated code.
* Added HAL entry point as workaround fix for random ESP32 crashes \(related to unreferenced float variable in ISR context\). 

Drivers:

* Most: updated to use new core based stream switcher.
* STM32F4xx: updates for BTT SKR 1.1 & 2.0 UART mode Trinamic stepper driver support.
* LPC176x: Added tentative support for BTT SKR E3 board including soft UART mode Trinamic TMC2209 drivers. Code by Dimitris Zervas, somewhat modified by Terje Io.
* ESP32: added driver support for ganged/auto squared axes and Trinamic SPI mode stepper drivers. Untested for now since hardware is not available.  
Added board map for xPro v5 controller with TMC5160 drivers. Untested.  
Fix for random Guru crashes when streaming gcode at high feedrates/accelerations.

Plugins:
* Trinamic: workaround for ESP32 enums always defaulting to 32bit(!)
* Networking, Bluetooth: updated for core stream switcher.

---

Core:

* Renamed unused function.

Plugins:

* Networking: "hardened" websocket code. Increased ftpd buffer for faster ftp downloads.
* SD Card: Added mount and unmount events to allow moving driver specific code out of the plugin.

Drivers:

* RP2040, Simulator: updated for core changes that should have been committed for build 20211203.
* ESP32, iMXRT1062 and MSP432E401Y: moved driver specific SD card mount/unmount code from the SD card plugin to _driver.c_.
* ESP32: improved SD card mount/unmount code. Added `$FU` command to be used to unmount the card before removing it. The `$FU` command may be removed later.
* LPC176x: Changed IRQ priorities in order to avoid random delays when sending lots of short movements \(e.g. when laser engraving\).  
Fixed bug in SD card driver code.

---

Build 20211203:

Core:

* Added flags for WebUI reconnect handling.
* For developers: Changed debug stream initialization to claim specific stream instance and moved init call to core.  
Enabling the debug stream is done in [grbl/config.h](https://github.com/grblHAL/core/blob/master/config.h).

Drivers:

* All: updated for core changes \(new flags\).
* iMRXRT driver: fixed regression causing UART mode to fail, driver issue [#28](https://github.com/grblHAL/iMXRT1062/issues/28).

Plugins:

* SD card plugin: Fixed issue that crashes the controller if a client disconnects/reconnects while a job is running.   
Added support for WebUI disconnect/reconnect without terminating a running job.

---

Build 20211130:

Core:

* Fixed regression affecting ESP32 driver, crashes the controller in some circumstances. One is starting a second WebUI when the first is executing movements.

Drivers:

* ESP32 driver: Added compiler flags to shut down some compiler warnings, increased main task stack size.
* LPC176x driver: Added option to enable I2C current control for Smoothieboard. _Not tested!_
* Some minor fixes in a couple.

Plugins:

* Networking plugin: Fixed issue in WebSocket daemon that resulted in some clients not accepting connection offered.

---

Build 20211128:

Core:

* Added functionality for serial stream registration and enumeration, allows plugins to claim stream(s) from free pool.
* Simplified registration of settings, alarms and error codes from driver and plugin code.
* Fixed `$$` settings report, it reported some configuration dependent settings when not available.
* Added pin id definitions for soft UART pins \(used by Trinamic TMC2209\) driver code.

Plugins, templates and drivers:

* Updated for core changes above.

Drivers:

* ESP32 driver: updated for ESP-IDF 4.3 API call change that prevented WiFi AP mode initialization. Rewrite of wifi code that used deprecated library calls.
* STM32F4xx driver: added tentative support for Trinamic stepper drivers for BTT SKR Pro 1.1 \(and 1.2 that can use the same map\) board. _Not tested!_

Plugins:

* Motors plugin: bug fixes for handling of ganged motors with Trinamic drivers. Added optional callbacks to low level driver code pre and post configuration \(per driver\).

---

Build 20211122:

Core:

* Fixed silly typedef mistake in setting struct, added settings `$392` and `$393` for spindle and coolant startup delays on safety door open.
* Removed Grbl v0.9 error messages.

Drivers:

* Added setting $308 for FTP port to use for networking capable drivers. Defaults to 21.
* Fixed WebUI processor clock speed report in ESP32 driver, added FTP port, board name and driver version to same.

---

Build 20211121:

Core:

* Added some data fields to the settings structures and modified a HAL API call signature.  
Due to this settings and HAL version numbers has been increased to 21 and 9 respectively.  
__NOTE:__ due to this settings will be reset to default values after an update. Backup and restore!
* Added `$8` setting for inverting direction signals for the second motor for ganged axes.  
__NOTE:__ This setting is applied _after_ inversion is performed according to the `$3` setting.  
__NOTE:__ I have only bench tested this for a couple of drivers, correct function should be verified after updating!
* Changed default value for `$4` stepper enable setting to invert all axes \(active low\).
* Added reason code \(flags\) to "Incompatible driver" message, delayed halt so other POS \(Power On Self-test\) messages is not lost.
* Some bug fixes in new ioports code.

Plugins:

* Updated _Bluetooth_, _Fans_, _Laser coolant_ and _Plasma_ plugins with settings for selecting aux port\(s\) to use.  
__NOTE:__ Port settings are added under the _Aux ports_ setting group even when the plugin has its own setting group.
* Updated SD card plugin to support manual tool change on `M6` \(only available if the driver supports it\).

Drivers:

* Updated many for additional features in enhanced ioports API.
* Updated drivers supporting ganged axes for the new `$8` setting.
* iMRXT1062 \(Teensy 4\): Added full ioports support for MCP3221 I2C ADC.
* Some bug/regression fixes.

---

Build 20211117:

Core:

* Enhanced [ioports API](http://svn.io-engineering.com/grblHAL/html/ioports_8h.html) with new calls for claiming ports and swapping pin to port mappings.
* Fixed bug #87 where executing G28/G30 with explicit motion when a motion mode was not active \(following a G80\) raised error 31.
* Replaced string symbol `GRBL_VERSION_BUILD` with numeric symbol `GRBL_BUILD`.  
The new symbol can be used by plugin code to check for functionality.

Plugins:

* Updated Bluetooth plugin for enhanced ioports API, bug fix.
* Updated Spindle plugin for dual spindle support, merged PR#4.
* Updated some to use the new `GRBL_BUILD` symbol.

Drivers:

* Updated many for define symbol changes (harmonization) and enhanced ioports API.
* iMRXT1062 \(Teensy 4\): Fixed bugs, one was a typo and one slowed down max USB streaming rate.
* RP2040 \(Pi Pico\): Added support for SD card and ioports API for aux output pins on Pico CNC board.
* STM32F3xx: Fixed typo and some compiler warnings.

---

Build 20211108:

Core:
* Renamed default hostnames/device names from _Grbl_ to _grblHAL_.
* Added early application of constraints for feedrate and spindle RPM overrides to avoid variable under/overflow. From PR#83.
* Added higher level \(than generic GPIO\) interrupt handler registration to HAL.
* Added optional MCU peripheral \(other than GPIO\) pin registration to HAL. For `$pins` report.
* Added region for [third party plugin](https://github.com/grblHAL/plugins#i-have-written-a-plugin-and-i-want-to-make-it-available-to-grblhal-users) init functions to _plugins_init.h_.

Plugins:
* Modified keypad plugin to claim strobe interrupt only when using I2C, added `KEYPAD_ENABLE 2` option to _my_machine.h_ to not claim it.
* Fixed botched include in Bluetooth plugin, added pin description to claimed UART pins.
* Made Modbus plugin configuration symbols settable from compiler command line, added pin description to claimed UART pins.

Drivers:
* Updated to provide peripheral pin info to `$pins` report.
* Some bug fixes.

---

Build 20211029:

Core:
* Fixed variable name typo.
* Renamed hal.stream.disable to hal.stream.disable_rx to better reflect function.

Plugins:
* Spindle plugin modified to support multiple Modbus clients.

Drivers:
* Updated for HAL function pointer rename, multiple Modbus clients where relevant.
* Added support for using UART8 as serial port in iMXRT1062 driver.

---

Build 20211024:

Core:
* Moved @ G59.3 probing event call earlier in code, added check for X and Y homed.
* Minor "bug fix" for real-time report, no longer sends `WCO` and `Ov` elements in the same response.

Plugins:
* A bit of cleanup in the websocket protocol code.
* Fixed some miscopied lines in the Trinamic driver from a manual merge of [PR#3](https://github.com/grblHAL/Plugins_motor/pull/3).

Drivers:
* Added map file for [GRBLHAL2000](https://github.com/Expatria-Technologies/grblhal_2000_PrintNC) board to iMRXT1062 driver \(Teensy 4.1\) From [PR#23](https://github.com/grblHAL/iMXRT1062/pull/23) by @andrewmarles

---

Build 20211019:

Core:
* Some minor changes to better support Trinamic drivers, probing and drivers/boards with limited number of control inputs \(cycle start, feed hold, ...\).

Plugins:
* Trinamic driver enhancements: Allow different StallGuard threshold settings for seek and locate phases, option to reduce acceleration during homing, bug fixes++  
  __NOTE__: _All_ plugin settings will be reset when updating if the Trinamic plugin is in use. Backup and restore.

Drivers:
* Improved TMC2209 UART support for the STM32F407 based BTT SKR 2.0 board. By @fitch22
* Added PicoBOB board support to RP2040 driver. By @andrewmarles

Templates:
* Corrections for Arduino builds, a few minor bug fixes.

---

Build 20211015:

Core:
* Improved `grbl.on_probe_fixture` event signature by adding pointer to the pending tool when fired when M6 is executing \(NULL if not\) and a flag indicating that the current XY-position is within 5mm of G59.3.
* A few minor fixes.

Plugins:
* I2C keypad: Switch to metric mode when jogging.
* WebUI: added option to store read-only webui files in flash.

Templates:
* Updated the [probe select](https://github.com/grblHAL/Templates/blob/master/my_plugin/probe%20select/my_plugin.c) example for the `grbl.on_probe_fixture` signature change and added an optional mode select parameter to M401.

---

Build 20211010:

Core:
* Added enum and associated get function for system defined named parameters.
* Added events (function pointers) for tool change mode 2 and 3 events \(can be used to switch between probes\) and for publishing active homing feedrate.

Drivers:
* Improved stream support in many drivers by adding support for _write_n_ characters. This is mainly used for outputting human readable settings descriptions.
* Added TMC2209 UART support to the STM32F407 based BTT SKR 2.0 board. By @fitch22

Plugins:
* Added support for reading files from flash based storage to http daemon support code.  
  If a file is not found in the SD-card _www_ folder an attempt will be made to locate in in flash.  
  Added `WEBUI_INFLASH` option to _my_machine.h_ for storing WebUI files \(_index.html.gz_ and _favicon.ico_\) in flash.  
  Note that if these files are found in the SD card _www_ folder they will be used instead.
* Improved Trinamic driver support both generally and for the TMC2209 silent stepstick.  
  The default feedrate for switching to SpreadCycle mode has been changed to 0 \(never switch\), use [M913](https://github.com/grblHAL/Plugins_motor) to set it or set the default value in [trinamic.h](https://github.com/grblHAL/Plugins_motor/blob/master/trinamic.h) by changing the `PWM_THRESHOLD_VELOCITY` symbol.  
  This is still work in progress, sensorless homing is most problematic and is likely to require tuning of several parameters. E.g. the slow approach feed rate should not be too slow - the default 25 mm/min certainly seems to be.

---

Build 20210928:

Core:
* Changed safety door/parking handling to be compliant with legacy Grbl - now a cycle start command has to be issued to resume after the door is closed.
* Added `$384` setting for controlling G92 offset persistence, set to `1` to disable persistence across a reboot, `0` to enable. Only available if [compatibility level](https://github.com/grblHAL/core/wiki/Compatibility-level) is < 2, default value is `0`.
* Improved `$help` command output and handling, added description to `$$=<n>` output.
* Moved the optional tool table in non-volatile storage \(typically EEPROM\) to above the core area. This allows a larger number of tools \(max. 16\) to be defined.  
__NOTE:__ If you have tool table support enabled before upgrading the current table will be lost and possibly also all other settings. Backup and restore!
* Added gcode parameter support. All [NIST RS274NGC version 3](https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3) parameters (see section 3.2.1) and most [LinuxCNC](http://www.linuxcnc.org/docs/html/gcode/overview.html#_parameters) parameters are supported.  
  The `$#=<n>` or `$#=<name>` commands can be used to output a parameter value. Replace `<n>` with a parameter number, `<name>` with a parameter name.  
__NOTE 1:__ Named parameters and parameters in the range 1 to 5160 are volatile and will not persist across a reboot.  
__NOTE 2:__ Space for the volatile parameters is allocated at run-time, available memory \(heap\) sets a limit to how many can be set.  
__NOTE 3:__ Maximum name length is 20 characters, maximum number of parameters that can be set in a block \(line\) is 10.
* Added gcode [expression](http://www.linuxcnc.org/docs/html/gcode/overview.html#gcode:expressions) support. This has to be enabled in [grbl/config.h](./config.h) by uncommenting `//#define NGC_EXPRESSIONS_ENABLE 1`. _Experimental_.  
__NOTE:__ Processors with limited memory may not compile with this enabled.

Drivers & plugins:

* Added [WebUI plugin](https://github.com/grblHAL/Plugin_WebUI) support for some networking capable boards.
* Updated some drivers for internal API changes. Some minor bug fixes.

---

Build 20210907:

Core:
* Improved HAL/core stream handling for plugin issued [realtime commands](http://svn.io-engineering.com/grblHAL/html/structgrbl__t.html#a2ff011529d0a7809a44c8609490e0306).
* Some minor bug fixes and added rudimentary settings validations relevant when switching board map files.

Drivers & plugins:
* Fixed regression in encoder plugin.
* Updated all drivers and relevant plugins for HAL/core API changes.
* Added PWM inversion option to [iMXRT1062 driver](https://github.com/grblHAL/iMXRT1062).
* Significantly improved [STM32F7xx driver](https://github.com/grblHAL/STM32F7xx), still work in progress.

---

Build 20210819:

Core:
* Added `$376` setting for designating ABC-axes individually as rotational.  
__NOTE:__ This setting is only available when N_AXIS is > 3 and will force a settings reset on an upgrade for such configurations. Backup and restore settings when upgrading!  
Scaling from inches to mm is disabled for axes designated as rotational, no other processing takes place.
* Added `$ESG` and `$ESH` system commands for outputting current setting definitions in [Grbl csv-format](https://github.com/gnea/grbl/tree/master/doc/csv) and grblHAL tab-format respectively.
Only settings valid in the active configuration will be outputted, driver and plugin specific settings will be added as well - even from well behaved third party code.
* Added setting descriptions to most core/driver/plugin settings. Third party drivers and plugins may also add descriptions to any settings implementented.  
NOTE: Drivers for processors with limited flash may not have the descriptions compiled in.
* Added `$SED=<n>`  for outputting a description for setting `<n>` (if available), e.g. issue `$SED=14` to get a description for `$14`. The description is formatted for sender use.

Drivers & plugins:
* Cleaned up/simplified many pin mapping files, mostly by moving pin to bit transforms to a common preprocessor file.
* Some general driver and plugin improvements, e.g. more flexible spindle PWM to pin mappings for STM32F4xx and STM32F7xx drivers.

---

Build 20210803:

Core:
* Added optional HAL entry point for stepper motor enumeration (axis vs. motor id). Used by motors plugin.
* Improved optional driver support files.

Drivers & plugins:
* Added F103RC support and a board map for the BTT SKR MINI E3 V2.0 to the STMF1xx driver.
* Added tentative plasma plugin support to the SAM3X8E \(Arduino Due\) driver.
* Updated motors plugin to match updated [Trinamic library](https://github.com/terjeio/Trinamic-library), added support for TMC2209++.
* Some minor bug fixes and improvements.

---

Build 20210726:

Core:
* Added encapsulation in stream handlers for realtime processing. This allows more flexible use of additional streams.
* Added optional debug stream to the HAL. Currently only used by the iMXRT1062 driver.
* Position will now be recalculated on steps/mm setting changes.
* Limited final `$TPW` retract on tool changes to Z home to avoid triggering Z-limit.
* Moved some common driver code snippets to the core to avoid duplication.

Drivers & plugins:
* Updated all for new stream encapsulation.
* Added option to use UART input for I2C keypad plugin. Added fan toggle support.
* Fixed some bugs and regressions.
* Added support for ganged/autosquared axes to RP2040 driver.

---

Build 20210707:

Core:
* Added realtime command `0x8A` for toggling fan 0 on/off. Requires the Fans plugin.
* Moved a bit of code to aid documentation structure.
* Added option for plugins to add pin description for claimed pins to `$pins` report.
* Simplified API for user added M-codes, backwards compatibility retained for now.

Drivers:
* Updated a number of drivers for new style pin handling (allows `$pins` report and simpler pin mapping files).
* Updated and verified driver for RP2040, mainly to support the PicoCNC board.
* Added/improved interrupt support for auxiliary inputs for both plugin code (via API) and for `M66`.
* Bug fixes.

Build 20210626:
* Standardized handling of motors for ABC- and ganged/squared axes, configuration moved to _my_machine.h_.  
First number of motors required is calculated, then ABC axes are added from bottom up and then ganged/squared axes from top down.  
E.g if the board map supports six motors then the following allocations will be made:  
A-axis and auto-squared Y axis: A-axis -> motor 4 and second Y-axis -> motor 5. Motor 6 pins may then be assigned to auxiliary I/O.  
A-axis, B-axis and ganged X-axis: A-axis -> motor 4 and, B-axis -> motor 5 and second X-axis -> motor 6. Motor 6 limit pin\(s\) if available may be assigned to auxiliary I/O.  
Auto-squared X and Y-axis: second X-axis -> motor 4 and second Y-axis -> motor 5. Motor 6 pins may then be assigned to auxiliary I/O.  
etc...  
__IMPORTANT:__ For those who have used auto-squared/ganged axes with previous builds be sure to check that the motors allocated matches the current wiring.
Tip: use the `$pins` system command to list the pin allocations when checking. Rewire as necessary. 
* Added ganged axis/auto-squaring support to some board maps for the [LPC176x](https://github.com/grblHAL/LPC176x) driver.
* Expanded on HAL entry points for stream communication and added [initial documentation](http://svn.io-engineering.com/grblHAL/html/hal_8h.html) for the HAL and parts of the core.
* Encapsulated UART/USB CDC code for many drivers, for most only the init function is now available for direct access from the outside. Simplified main driver code and plugins using streams.  
__NOTE:__ For driver developers: `hal.stream.get_rx_buffer_available` has been renamed to `hal.stream.get_rx_buffer_free` as it was ambiguous \(free space vs. available characters\).
* Added preview version of [Bluetooth plugin](https://github.com/grblHAL/Plugins_Bluetooth/) - allows auto configuration and auto stream switching for drivers/boards that supports it.
* Added number of auxiliary I/O ports available to `$I` command response.
* Added value read from last `M66` to the first real-time report following the read. The element `|In:<value>` is used for the report. If the value reported is `-1` the read failed.

---

Build 20210608:
* Fixes for `$70` setting handling, networking services. Only compile time enabled services \(or protocols\) can now be configured.

---

Build 20210604:
* Added some HAL entry points and properties, shared file for mapped step and dir output.
* Added `$pins` system command, for listing current pin assignments. Work in progress, only supported by a couple of drivers. For now only plain GPIO pins are listed.
* Some minor bugs fixed. 

---

Build 20210515:

* Fixed G43 "bug" by changing to LinuxCNC specification. NIST specification is ambiguous.
* Added acceleration override handling, to be used by OpenPNP plugin M-code.

---

Build 20210505:

* Some bug fixes: relative canned cycles repeat error, comment handling...
* Check mode changes to allow use with SD card streaming.
* Internal buffer changes for reducing RAM footprint. Some HAL extensions.

---

Build 20210314:

* Added support for tool change protocol to networking protocols using shared stream buffer.
* Added `$S` system command for toggling single step mode, when enabled cycle start has to be issued after each block.
* Some bug fixes.

---

Build 20210207:

* Added `#define BOARD_MY_MACHINE` option in _my_machine.h_ for building using _my_machine-map.c_, this for simpler handling of user defined pin mappings.  
_my_machine-map.c_ is __*not*__ part of the distributed source and must by added to the project by the user before enabled, typically by copying an existing map file.
* Added HAL layer on top of Trinamic driver low-level code. Unified Triniamic plugins into single plugin.  
This is still work in progress, testers wanted.
* Added core support for up to four limit switches per axis.  
Added `$LEV` command for outputting report containing which limit or control switch(es) caused the last event.  
Report format:  
`[LASTEVENTS:<control signals>,<min>,<max>,<min2>,<max2)]`  
Where `<control signals>` field may contain controls signal letters (`H`, `S` etc.) and the rest axis letters for the corresponding limit switches inputs.
* More settings subsystem changes and refactored $-system commands parser.  
There are some API changes related to this that may affect user defined plugins.
* Added $7 setting for option "Spindle off with zero speed".
* Added `|FW:grblHAL` element to full real-time report requested by sending `0x87`. 
* Simplified settings handling.
* No longer sends any messages to networking streams on connect.
* Added [driver](drivers/STM32F3xx) for STM32F303 based Blackpills.
* Updated support for Trinamic TMC5160 drivers, currently for the SKR 1.x boards \(LPC176x driver\) - testing i progress.
* Initial support for Trinamic TMC5160 drivers added, currently for the SKR 1.x boards \(LPC176x driver\) - not yet tested.
* Added handling for motor fault signal to the core, similar to E-stop handling. Added new alarm and error code for this.
* Changed to clear homed status on a soft reset only if machine was in motion. Added setting flag for always keeping homed status on soft reset to `$22` setting.  
__NOTE:__ This change is experimental and might be changed or reverted. Please report any problems related to this.
* Updated gcode parser \(grbl/gcode.c\) to use bitfields structs instead of bitfield variables.  
Done to improve readability and for easier debugging.  
Note that this is a major change and there is a non-zero risk that mistakes has been made.
* Changed signature of user mcode validation function to use a bitfield union for value words available.  
Removed the need for user mcode parameter words to have an associated value. This means that [user mcode](https://github.com/terjeio/grblHAL/tree/test/templates) implementations now must check this locally.  
If no associated value is provided the corresponding value in the value struct is set to `NAN` (Not A Number) for floats and all bits set to 1 for integers.
* Refactored Trinamic driver code, added initial support for TMC2209. Work in progress.
* Removed the need to copy the core grbl and plugin code to the driver chosen, this is now kept in sync with the master Subversion repository automatically.
* Added initial support for RADDS 1.6 board to SAM3X8E driver \(Arduino Due\). Untested!
* Added C-axis support to iMXRT1062 driver \(Teensy 4.x\).  
Untested and none of the current board maps has the needed pins defined.
* Added alarm and error message for power on self-test \(POS\) failure.  
If POS fails only $-commands are accepted.
* Work in good progress for Trinamic TMC2209 driver support \(UART mode\).  
Processor/board specific driver code has to be added for this, currently testing with STM32F446 and Nucleo-64 breakout board.
* Renumbered setting groups for more logical sorting (by id).
* Harmonized probing code across drivers for planned future extensions.
* Added additional I/O support for the [Teensy 4.1 T41U5XBB board](https://github.com/phil-barrett/grbl-teensy-4) \(iMXRT1062 driver\), 3 outputs and 4 inputs available via `M62` - `M66`.  
Not that the result from reading inputs with `M66` cannot be used in a gcode program in any meaningful way.
* Fixed excessive step pulse jitter in STM32F4xx driver.
* Added [more options](https://github.com/terjeio/grblHAL/wiki/Report-extensions#controller-information-extensions) to the `NEWOPT` tag in the extended `$I` report.
* Error 7 is no longer issued on startup if non-volatile storage \(Flash/EEPROM/FRAM\) for settings is not available.
* [Alarm substate](https://github.com/terjeio/grblHAL/wiki/Report-extensions#realtime-report) \(if available\) is always added to the real-time report if a [complete report](https://github.com/terjeio/grblHAL/wiki/For-sender-developers#single-character-real-time-commands) is requested by sending `0x87`.
* Added input signal and handling for limit switches override.  
The pin is pulled high and requires a normally open \(NO\) push switch for operation. When closed limit pins are excluded from the status report and alarm 12 will not be raised if a limit switch is asserted _on a soft reset_ when "Hard limits" and "Strict mode" is enabled with `$21`.
This allows normal operation so that a manual pull-off can be done before e.g. homing the machine.  
Currently only the iMXRT1062 \(Teensy 4.x\) driver has support for this, for now by reassigning the safety door input when this is not used for its intended purpose.  
__NOTE:__ A override will _not_ affect handling of homing and limit switch events elsewhere.
* Now adds `ODO` to `NEWOPT` tag values if odometer data is available.
* Updated _[my_plugin.c](templates/my_plugin.c)_ [template](templates/README.md) with settings details for `$HELP` and `$ES`/`$EG` enumerations.
* Settings/setting groups handling enhanced, moved some to plugins and added sorting (requires enough heap).
* Removed external dependencies by adding driver source/USB blob to LPC176x driver.
* Enhanced and improved ModBus support code for VFD spindle, added settings for baud rate and receive timeout.
* Added support for enumeration of and help for driver and plugin provided settings and setting groups.
* Moved board selection etc. to [CMakeLists.txt](drivers/ESP32/CMakeLists.txt) for [ESP32 driver](drivers/ESP32/README.md) for simpler configuration.
* Fixed regression for VFD spindle code, should now be able to run tests.
* Added build configurations for processor variants and Nucleo-64 boards for the [STM32F4xx driver](drivers/STM32F4xx/README.md).
* Added initial board map file for [BTT SKR 1.4 Turbo board](https://www.bigtree-tech.com/products/btt-skr-v1-4-skr-v1-4-turbo-32-bit-control-board.html) to the [LPC176x driver](drivers/LPC1769/README.md) including build configuration for bootloader compatible executable.
* Added polling of limit switches to the LPC176x driver, enabling hard limits is now possible.
* Added `$` commands for getting details about alarm codes, error codes, settings and settings groups.  
`$EA` - enumerate alarm codes.  
`$EE` - enumerate error codes.  
`$ES` - enumerate settings.  
`$EG` - enumerate setting groups.  
`$E*` - enumerate all above.  
The output from these is intended for sender developers and can be used instead of loading this information from .csv files.  
The settings enumeration contains additional information such as group assignment, datatype and format, value list for bitfields, min allowed value and max allowed value. 
For now descriptions of the settings are not included.  
__NOTE:__ This is a preview version, format and group codes may change for settings and setting groups.
* Added `$HELP` command and `$$<n>` command for listing information about a specific setting.  
`$HELP` on its own prints arguments that can be used with `$HELP`.  
`$HELP Commands` - print `$` commands available with a short description.  
`$HELP Settings` - print information about all available settings.  
`$HELP <argument>` print information about settings from the setting group provided in `<argument>`. E.g. `$HELP Spindle` will print information about spindle settings.  
__NOTE:__ do _NOT_ issue these commands from a sender MDI as the output may crash it, output is in plaintext and thus intended for use from a terminal only.
__NOTE:__ Settings data format has been changed and settings will be reset to default on update. Backup and restore.
* Moved `#define` values to settings for auto square failure distances:  
`$347` - default value from `DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT` \(5%\) in grbl/config.h.  
`$348` - default value from `DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN` \(2.5mm\) in grbl/config.h.  
`$349` - default value from `DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX` \(25mm\) in grbl/config.h.
* Added settings and functionality for moving the second axis up to &plusmn;2mm after successful auto squaring to compensate for any switch alignment error.  
The settings is per axis, `$17n` where `n` is the axis index: `n` = `0` -> X axis, `1` -> Y axis, ...  
Note that settings values will only be reported for axes with dual motors installed and configured for auto squaring. 
* Added setting `$345` for pull-off rate from tool length sensor used for tool change. Default value is 100mm/min.
* Added setting flag to `$10` for enabling override of _Homing on startup required_ \(if enabled\) by a soft reset.
* Blocked loophole where machine could be unlocked by issuing a single axis homing command when _Homing on startup required_ is enabled.  
Alarm 11 will now be reissued until all axes configured for homing are homed.
* Added software debounce for the safety door switch to STM32 drivers.
* "Hardened" parking functionality. It should now tolerate a bouncy door switch and multiple closing/reopenings of the door during retract/restore.  
__NOTE:__ Not extensively tested. Use with care!
* Added `$I+` system command. This can be used when [compatibility level](https://github.com/terjeio/grblHAL/wiki/Compatibility-level) is > 0 to get the extended version including the current compatibility setting.
* Added basic support for separating limit switches from homing switches in the core. If a driver does not handle separate inputs for these the core "connects" the homing switches to the limit switches in code.
* Improved auto squaring. If a limit switch is engaged when homing starts the axis will be moved pull-off distance * 5 away from them. If still engaged homing will fail.  
__NOTE:__ Auto squaring is currently only tested with a simulator. Use with care!

---

Build 20201103:

* Added data structures for spindle encoder/spindle sync to the core. Used by drivers supporting spindle sync.
* Updated spindle sync code for MSP432 and added spindle sync capability to iMXRT1060 and STM32F4xx drivers.  
__NOTE:__ Spindle sync support is still in alpha stage! The current code has only been tested with a simulator.
* Fixed bug that could lead to settings storage area fail to reinitialize properly when corrupted.
* Moved some symbols in preparation for adding $-settings for them.
* Improved rewind capability (on `M2`) of SD card plugin. Added `$` command for dumping SD card file contents to output stream.

---

Build 20201020:

__NOTE:__ Settings data format has been changed and settings will be reset to default on update. Backup and restore.

* Added support for STM32F446 based Nucleo-64 boards to STM32F4xx driver.
* Fix for regression that set laser mode as default - `$32=1`. Check that this is correct after restoring settings from backup. 
* Added new settings option `$341=4` for ignoring `M6` tool change command.
* Bug fix for `M61Q0` - returned error previously.
* Changed clearing of tool length offset reference on homing to be done only if relevant axis is/axes are homed.
* Fixed check for running startup scripts on homing to check that all axes configured for homing are actually homed.
* Changes to message display from `(MSG,..)` comments in gcode.  
`(MSG,..)` strings will be sent back to the sender in sync with gcode execution.
* Added `*` prefix to NVS storage type in `$I` report if buffered, e.g: `[NEWOPT:*EEPROM,ES,TC]`
* Increased heap allocation for nearly all drivers.  
Memory from heap is used for the NVS buffer and for temporary storage of `(MSG,..)` strings.  
* Added option for adding substates to the `Run` state in the real time report to `$11` setting \(bit 11\).  
If enabled `Run:2` will be reported when a probing motion is ongoing, this can be used by senders to provide a simple probe protection scheme.  
* Another refactoring of the settings subsystem, this time for handling plugin settings.
Plugins settings storage space is now dynamically allocated and handled locally by the plugin code, this allows user defined plugins to add settings too!
Ten setting codes are reserved for user defined plugins.
* Added template for [basic plugin with two settings](templates/README.md).
* HAL pointers refactored for code readability, many moved to separate structures for each subsystem. Added typedefs.
* Some minor bug fixes and other code readability improvements. Prepared for switch to 16-bit CRC for settings data validation.

---

Build 20200923:

* Added support for STM32F411 based Blackpill boards to STM32F4xx driver.
* Initial changes to ESP32 driver to allow compilation with PlatformIO, added my_machine.h for this. Note that my_machine.h is not used if compiling with idf.py.
* Added home position to `$#` ngc report, e.g. `[HOME,0.000,0.000,0.000:7]` - means all axes are homed. Position is reported in machine coordinates. `:7` in the example is an axis bitfield, the reported value is for which axes are homed: bit 0 is Z, 1 is X etc. `:0` = no axes homed.
* "Hardened" the new tool change functionality even more. Initial changes for multi-axis tool reference offset made.  
An empty message will now be sent when tool change is complete, this to clear any tool change related message in the sender.
* Added call to [weak](https://en.wikipedia.org/wiki/Weak_symbol) `my_plugin_init()` function at startup, name your [plugin](https://github.com/terjeio/grblHAL/tree/master/plugins) init function `void my_plugin_init (void)` and there is no need to change any grblHAL source files to bring it alive.  
Use this feature for your private plugin only, multiple public plugins using this name cannot coexist!
* Some changes to improve code readability and added strict check for `G59.x` gcodes.

---

Build 20200911:

* Core refactored for better support for non-volatile storage. Some HAL entry points renamed for readability and moved to a new data structure.
* Added plugin for axis odometers. This logs total distance traveled and machining time to EEPROM/FRAM.  
[FRAM](https://www.electronics-notes.com/articles/electronic_components/semiconductor-ic-memory/fram-ferroelectric-ram-memory.php) is recommended for storage as it is faster and can sustain a larger number of write cycles. FRAM chips are sold in packages that is pin compatible with EEPROM.  
__NOTE:__ Currently for review and for now only for the iMRXT1061 \(Teensy 4.x\) driver. It will _not_ be available in configurations that stores non-volatile data to flash.
* "Hardening" of new [manual tool change](https://github.com/terjeio/grblHAL/wiki/Manual,-semi-automatic-and-automatic-tool-change) functionality. 
* Improved auto squaring algorithm in core.
* Enhanced some plugins so they can coexist.

--- 

Build 20200830:

* Improved tool change functionality, a status report is forced on end of tool change to inform change is completed.
* Fix for iMRXT1062 homing failure when serial over USB is used.
* Standardized serial over USB `#define` macro across drivers to `USB_SERIAL_CDC`.

---

Build 20200818:

* Part two of large overhaul of configuration system.

Driver configuration has been moved from _driver.h_ to _my_machine.h_ and options has been made available to be defined as compiler defined symbols/macros with the `-D` compiler command line option. In order to allow compiler defined symbols to override _my_machine.h_ settings the symbol `OVERRIDE_MY_MACHINE` should always be added to the command line. See [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for more information. Available driver options in _my_machine.h_ can now be listed from the driver ReadMe page.

---

Build 20200813:

* "Hardened" step pulse generation when delay is enabled. The delay will now only be added on direction changes.
The minimum possible delay is dependent on many factors and will be adjusted to match what the processor is capable of.
By default driver code is calibrated for a 2.5 &micro;s delay and may deviate a bit for other settings and configurations. The actual delay should be checked with an oscilloscope when high step rates are used.
__NOTE:__ A delay is only added if AMASS is disabled, when AMASS is enabled \(it is by default\) there is always a implicit delay on direction changes.

---

Build 20200811:

* Part one of large overhaul of configuration system.
This has been done to allow a global configuration file and/or setting compile time options as compiler symbols/macros with the `-D` compiler command line option.
* Networking \(cabled ethernet\), SD Card and I2C keypad plugin support added to [Teensy 4.x driver.](drivers/IMXRT1062) A Teensy 4.1 is required for networking.
* Tool number range changed from 8-bit \(0-255\) to 32-bit \(0-4294967294\). Note that if the optional tool table is enabled the max tool number is limited by number of entries in the tool table.
* M60 and corresponding pallet shuttle HAL entry point added. No driver support for this yet.

_Configuration system changes:_

Symbols \(#define macros\) that are unlikely to be changed or is used for conditional compilation has been moved from [grbl/config.h](grbl/config.h) to [grbl/grbl.h](grbl/grbl.h).
Symbols used for conditional compilation can be overridden by `-D` compiler command line options or by editing [grbl/config.h](grbl/config.h).

[grbl/defaults.h](grbl/defaults.h) is now used for default values for settings that can be changed at run-time with the `$$` command. __Important:__ grbl/defaults.h should only be changed when adding new settings.

[grbl/config.h](grbl/config.h) is now used for overriding default values in grbl/grbl.h or grbl/defaults.h. Out of the box all overridable symbols are commented out, uncomment and change as needed to override default definitions in grbl/grbl.h or grbl/defaults.h.

These changes are part of a long term plan to create a user friendly front end for configuring and building grblHAL.

---

Build 20200805:
* **Important:** settings has been changed again and settings will be restored to defaults after updating. Backup & restore!
* Added tool change handler for [manual tool change](https://github.com/terjeio/grblHAL/wiki/Manual,-semi-automatic-and-automatic-tool-change) on M6. Four different modes available, selectable with [`$341` setting](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings#manual-tool-change-settings).
* STMF4xx driver updated for optional I2C EEPROM plugin support.

---

Build 20200722:
* **Important:** settings version has been changed again and settings will be restored to defaults after updating. Backup & restore! 
* Changed step pulse width and delay settings from int to float and reduced minimum allowed value to 2 microseconds<sup>1</sup>. Useful for very high step rates.
* New plugin for [quadrature encoder input](https://github.com/terjeio/grblHAL/issues/73#issuecomment-659222664) for up to 5 encoders \(driver dependent\). Can be used to adjust overrides and has rudimentary support for MPG functionality. Work in progress and the iMXRT1062 \(Teensy 4\) driver is currently the only driver with low-level support for this (one encoder).
* New plugin for [ModBus VFD](https://github.com/terjeio/grblHAL/issues/68) spindle controllers. Untested and with limited driver support in this build.
* Added setting, `$340` for spindle at speed tolerance \(percent\). If spindle fails to reach speed within limits in 4 seconds alarm 14 will be raised. Set to 0 to disable. Availability driver dependent.
* All [new settings](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings) are now possibly to set independent of the [compatibility level](https://github.com/terjeio/grblHAL/wiki/Changes-from-grbl-1.1#workaround) except some settings that has flags added to them. The added flags will not be available at all compatibility levels. A new command, `$+`, can be used to list all settings independent of compatibility level.
* Internal changes to settings data in order to simplify automatic migration on changes. Automatic migration is on the roadmap.

<sup>1</sup> Note that several factors may affect the accuracy of these settings such as step output mode, number of axes defined and compiler optimization settings.
A new #define, `STEP_PULSE_LATENCY`, has been added to driver.h for those drivers that requires it so that fine tuning can be done. I may move this to a setting later.
In setups where very high step rates are used the actual step pulse width should be confirmed with an oscilloscope.

The step pulse delay has not been fine tuned yet as setting this to a value > 0 is not normally needed as there seems to be a implicit delay on direction changes when AMASS is enabled.

__Note:__ high step rates \(e.g. above 80 kHz\) cannot be achieved with the step pulse setting set to the default 10 microseconds. This has to be reduced so that both the on and off time is within specifications of the stepper driver. At 100 kHz the time available for a pulse \(on + off\) is 10 microseconds.

__Note:__ The SAMD21 \(MKRZERO\) driver needs updating in for it to allow short step pulses. My mistake was to use some Arduino code so that the Arduino pin numbers could be used for pin mappings. This adds around 500 ns per axis of overhead... To be fixed later.

For the curious: I have managed to achieve a 400 kHz step rate with the iMXRT1062 before everything breaks down, this with a command entered from MDI. I have not tested this with a running program, but I am pretty sure that a step rate at or above 200 kHz is sustainable.

---

2020/06/18: Added driver for STM32F4xx [Black Pill](https://www.cnx-software.com/2019/12/24/stm32-black-pill-board-features-stm32f4-cortex-m4-mcu-optional-spi-flash/), code modified by @shaise from the STM32F1xx driver. This is the first driver provided by someone else than me, thanks for that.

This driver is a candidate along with the IMXRT1062 \(Teensy 4.x\) driver to get spindle sync support. I have a [NucleoF411RE development board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) on order and will look into adding a pin mapping for that when it arrives. 

---


Build 20200603:
* **Important:** settings version has been changed and settings will be restored to defaults after updating. Backup & restore! 
* Optimizations for ring buffer handling in planner and step generator.
* New optional input signal for probe connected status, driver support will be added later to selected drivers.
* Automatic reporting of tool length offset \(`[TLO:...]`\) when changed.
* Support for [G5](http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G5-Cubic-Spline) \(cubic spline\) added.
* `G43.x`, `G49` and `G92` added to parser state report.
* `G76` [threading cycle](https://hackaday.io/project/165248-mini-lathe-emco-compact-5-cnc-conversion) refactored.
* \(Re\)added `REPORT_PROBE_COORDINATES` and `TOOL_LENGTH_OFFSET_AXIS` [configuration](grbl/config.h) options, the latter available when `COMPATIBILITY_LEVEL` > 2.
* Improved backwards compatibility with vanilla grbl, e.g. G92 and tool offset\(s\) will be lost on a soft reset. Dependent on `COMPATIBILITY_LEVEL` setting.
* Board name added to `$I` report if provided by driver.
* [Grbl-Sim](https://github.com/grbl/grbl-sim) ported to grblHAL as a [driver](drivers/Simulator). Added telnet support++. Can be used to test senders. Note: currently only compiled/tested for Linux.
* Some minor bug fixes.

---

Build 20200503: Added configuration flag for manual homing. \(Re\)added compile time option `ENABLE_SAFETY_DOOR_INPUT_PIN` for [safety door switch](https://github.com/terjeio/grblHAL/blob/master/grbl/config.h), default is now disabled. Some bug fixes and "hardening" of code.

---

Added some [template code](./templates/README.md) to aid customizations such as driver support for M62 - M68 M-codes mentioned below.

---

Build 20191222: Added digital and analog output support to the core \(and HAL\) as per [linuxcnc specifications for M62 - M68](http://linuxcnc.org/docs/html/gcode/m-code.html#mcode:m62-m65), number of outputs available \(if any\) is driver dependent. Adding support for these M-commands makes it fairly easy to add driver code \(for up to 256 outputs\) as parsing and synchronization is taken care of by the core.

---

Build 20191215: Moved spindle RPM linearization to $-settings, option needs to be enabled in config.h - driver support required. Optimized EEPROM allocation handling. WebUI support for ESP32 driver improved.  

MSP432 driver enhanced for spindle linearization app in the pipeline \(for Windows only - needs input from spindle encoder\), more work done on closed loop spindle RPM control and spindle synchronized motion - still at experimental stage.

__NOTE:__ settings version number has been increased so settings will be reset to default after update, make a backup first!

---

Added [#define COMPATIBILITY_LEVEL to config.h](grbl/config.h) for backwards compatibility with Grbl v1.1 protocol definition, this for enabling the use of more GCode senders. Please raise an issue if your sender still does not behave well after setting this as the current implementation does not yet disable all extensions, notably [new $xx settings](doc/markdown/grblHAL%20extensions.md#settings).

G76 threading support added to grblHAL in combination with the [MSP432 driver](drivers/MSP432/README.md). Extensive testing is required before it can be regarded as safe.

**WARNING!** This is a potentially dangerous addition. Do NOT use if you do not understand the risks. A proper E-Stop is a must, it should cut power to the steppers and if possible engage any spindle brake. The implementation is based on the [linuxcnc specification](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G76-Threading-Canned). Please note that I am not a machinist so my interpretation and implementation may be wrong!

G76 availability requires a spindle encoder with index pulse, grblHAL configured to [lathe mode](doc/markdown/settings.md#opmode) and tuning of the spindle sync PID loop.  
__NOTE:__ Feed hold is delayed until spindle synced cut is complete, spindle RPM overrides and CSS mode disabled through the whole cycle. 
