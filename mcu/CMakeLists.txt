set(PICO_BOARD pico CACHE STRING "Board type")
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

include_directories(tinyusb/src include)

# give the project a name (anything you want)
project(laft C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(laft)

# must match with executable name and source file names
target_sources(laft PRIVATE src/main.c)

# create map/bin/hex file etc.
pico_add_extra_outputs(laft)
