# Every .c file in the firmware is listed here. Add new modules to SRCS.
# (The "main" component automatically links against all ESP-IDF components,
#  so no REQUIRES list is needed while everything lives in main/.)
idf_component_register(
    SRCS
        "main.c"
        "board.c"
        "fsr.c"
        "imu.c"
        "acq.c"
        "detect.c"
        "logger.c"
    INCLUDE_DIRS "."
)
