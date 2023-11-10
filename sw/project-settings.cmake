# TODO: 项目名称
set(PROJECT_NAME "wwMotor")

# TODO: 目标MCU
set(TARGET_MCU "STM32G431RBT6")

option(USE_LL_LIB "Enable LL library" ON)
option(USE_HAL_LIB "Enable HAL library" ON)
option(USE_CMSIS_DSP_LIB "Enable CMSIS DSP library" ON)
option(USE_SYSTEM_VIEW "Enable Segger SystemView library" OFF)

set(SYSCALL "UART")
set(SYSCALL "RTT")
set(LOG_LEVEL "INFO")

# set(OS "threadx")
set(OS "nortos")
# set(OS "freertos")

# set(OS_PORT_FREERTOS_MEM_MANG "heap4")

# set(BSP_PORT "stm32h750")
# set(BSP_PORT "stm32g4xx")
