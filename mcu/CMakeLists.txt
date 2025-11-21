# cmake version
cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# give the project a name (anything you want)
project(laft C CXX ASM)

pico_sdk_init()

add_executable(laft)
target_sources(laft PRIVATE
  src/main.c
  src/usb_descriptors.c
)
target_include_directories(laft PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/pico
  include/
)

target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_ENTER_USB_BOOT_ON_EXIT=1)
target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_PRINTF_ALWAYS_INCLUDED=1)

target_link_libraries(${PROJECT_NAME}
  tinyusb_device
  tinyusb_board
  pico_multicore
  pico_stdlib
)

pico_enable_stdio_usb(${PROJECT_NAME} 0) # Incompatible with using tinyusb manually
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(laft)
