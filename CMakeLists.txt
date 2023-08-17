add_library(grbl INTERFACE)

target_sources(grbl INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/scara.c
 ${CMAKE_CURRENT_LIST_DIR}/grbllib.c
 ${CMAKE_CURRENT_LIST_DIR}/coolant_control.c
 ${CMAKE_CURRENT_LIST_DIR}/nvs_buffer.c
 ${CMAKE_CURRENT_LIST_DIR}/gcode.c
 ${CMAKE_CURRENT_LIST_DIR}/machine_limits.c
 ${CMAKE_CURRENT_LIST_DIR}/messages.c
 ${CMAKE_CURRENT_LIST_DIR}/modbus.c
 ${CMAKE_CURRENT_LIST_DIR}/motion_control.c
 ${CMAKE_CURRENT_LIST_DIR}/my_plugin.c
 ${CMAKE_CURRENT_LIST_DIR}/nuts_bolts.c
 ${CMAKE_CURRENT_LIST_DIR}/override.c
 ${CMAKE_CURRENT_LIST_DIR}/planner.c
 ${CMAKE_CURRENT_LIST_DIR}/protocol.c
 ${CMAKE_CURRENT_LIST_DIR}/report.c
 ${CMAKE_CURRENT_LIST_DIR}/settings.c
 ${CMAKE_CURRENT_LIST_DIR}/sleep.c
 ${CMAKE_CURRENT_LIST_DIR}/spindle_control.c
 ${CMAKE_CURRENT_LIST_DIR}/state_machine.c
 ${CMAKE_CURRENT_LIST_DIR}/stream.c
 ${CMAKE_CURRENT_LIST_DIR}/stepper.c
 ${CMAKE_CURRENT_LIST_DIR}/system.c
 ${CMAKE_CURRENT_LIST_DIR}/tool_change.c
 ${CMAKE_CURRENT_LIST_DIR}/alarms.c
 ${CMAKE_CURRENT_LIST_DIR}/errors.c
 ${CMAKE_CURRENT_LIST_DIR}/ngc_params.c
 ${CMAKE_CURRENT_LIST_DIR}/ngc_expr.c
 ${CMAKE_CURRENT_LIST_DIR}/ngc_flowctrl.c
 ${CMAKE_CURRENT_LIST_DIR}/regex.c
 ${CMAKE_CURRENT_LIST_DIR}/ioports.c
 ${CMAKE_CURRENT_LIST_DIR}/vfs.c
 ${CMAKE_CURRENT_LIST_DIR}/corexy.c
 ${CMAKE_CURRENT_LIST_DIR}/wall_plotter.c
)
