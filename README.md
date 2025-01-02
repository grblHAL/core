## grblHAL ##

Latest build date is 20250102, see the [changelog](changelog.md) for details.  

> [!NOTE]
>  A settings reset will be performed on an update of builds prior to 20241208. Backup and restore of settings is recommended.  

> [!NOTE]
>  Build 20240222 has moved the probe input to the ioPorts pool of inputs and will be allocated from it when configured.
The change is major and _potentially dangerous_, it may damage your probe, so please _verify correct operation_ after installing this, or later, builds.

---

A web app for [building for some drivers](http://svn.io-engineering.com:8080/) is now available, feedback will be appreciated.

grblHAL has [many extensions](https://github.com/grblHAL/core/wiki) that may cause issues with some senders.
As a workaround for these a [compile time option](https://github.com/grblHAL/core/wiki/Changes-from-grbl-1.1#workaround) has been added that disables extensions selectively. 

> [!NOTE]
> grblHAL defaults to normally closed \(NC\) switches for inputs, if none are connected when testing it is likely that the controller will start in alarm mode.  
> Temporarily short the Reset, E-Stop and Safety Door<sup>4</sup> inputs to ground or invert the corresponding inputs by setting `$14=73` to avoid that.  
> Please check out [this Wiki page](https://github.com/grblHAL/core/wiki/Changes-from-grbl-1.1) for additional important information.

Windows users may try [ioSender](https://github.com/terjeio/Grbl-GCode-Sender), binary releases can be found [here](https://github.com/terjeio/Grbl-GCode-Sender/releases).
It has been written to complement grblHAL and has features such as proper keyboard jogging, advanced probing, automatic reconfiguration of DRO display for up to 6 axes, lathe mode including conversational G-Code generation, 3D rendering, macro support etc. etc.

---

grblHAL is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling and is based on the [Arduino version of grbl](https://github.com/gnea/grbl). It is mainly aimed at ARM processors \(or other 32-bit MCUs\) with ample amounts of RAM and flash \(compared to AVR 328p\) and requires a [hardware driver](https://github.com/grblHAL/drivers) to be functional.
Currently drivers are available for more than 15 different processors/processor families all of which share the same core codebase.

grblHAL has an open architecture allowing [plugins](https://github.com/grblHAL/plugins) to extend functionality.
User made plugins can be added to grblHAL without changing a single file in the source<sup>1</sup>, and allows for a wide range extensions to be added.
New M-codes can be added, space for plugin specific settings can be allocated, events can be subscribed to etc. etc.  
Adding code to drive an ATC, extra outputs or even adding a UI<sup>2</sup> has never been easier. You can even add your own [driver](https://github.com/grblHAL/Templates/tree/master/arm-driver) if you feel so inclined.

HAL = Hardware Abstraction Layer

The controller is written in highly optimized C utilizing features of the supported processors to achieve precise timing and asynchronous operation.
It is able to maintain up to 300kHz<sup>3</sup> of stable, jitter free control pulses.

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and some canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

grblHAL includes full acceleration management with look ahead. That means the controller will look up motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

This is a port/rewrite of [grbl 1.1f](https://github.com/gnea/grbl) and should be compatible with GCode senders compliant with the specifications for that version. It should be possible to change default compile-time configurations if problems arise, eg. the default serial buffer sizes has been increased in some of the [drivers](https://github.com/grblHAL/drivers) provided.

<sup>1</sup> This feature is only to be used for private plugins, if shared then a single call must be added to the driver code of the target processors.   
<sup>2</sup> I do not usually recommend doing this, and I will not accept pull requests for any. However I may add a link to the github repository for any that might be made.  
<sup>3</sup> Driver/processor dependent.  
<sup>4</sup> Not enabled by default if building from source, but may be enabled in prebuilt firmware.

***

#### Supported G-Codes:

```
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G65*****, G92, G92.1
  - Additional Non-Modal Commands: G10L1*, G10L10*, G10L11*
  - Motion Modes: G0, G1, G2****, G3****, G5, G5.1, G38.2, G38.3, G38.4, G38.5, G80, G33*
  - Canned cycles: G73, G81, G82, G83, G85, G86, G89, G98, G99
  - Repetitive cycles: G76*
  - Feed Rate Modes: G93, G94, G95*, G96*, G97*
  - Unit Modes: G20, G21
  - Scaling: G50, G51
  - Lathe modes: G7*, G8*
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43*, G43.1, G43.2*, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30, M60
  - Coolant Control: M7, M8, M9
  - Spindle Control: M3, M4, M5
  - Tool Change: M6* (Two modes possible: manual** - supports jogging, ATC), M61
  - Switches: M48, M49, M50, M51, M53
  - Input/output control***: M62, M63, M64, M65, M66, M67, M68
  - Modal state handling*: M70, M71, M72, M73
  - Return from macro*****: M99
  - Valid Non-Command Words: A*, B*, C*, D, E*, F, H*, I, J, K, L, N, O*, P, Q*, R, S, T, U*, V*, W*, X, Y, Z

  * driver/configuration dependent. W axis only available when ABC axes are remapped to UVW or when lathe UVW mode is enabled.
  ** requires compatible GCode sender due to protocol extensions, new state and RT command.
  *** number of inputs and outputs supported dependent on driver implementation.
  **** supports multi turn arcs from build 20220718.
  ***** requires keypad macros plugin or SD card plugin. Nesting is not allowed.
```

G/M-codes not supported by [legacy Grbl](https://github.com/gnea/grbl/wiki) are documented [here](https://github.com/grblHAL/core/wiki/Additional-G--and-M-codes).

Some [plugins](https://github.com/grblHAL/plugins) implements additional M-codes.

---
20250102
