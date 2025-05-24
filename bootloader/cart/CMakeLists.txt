# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

set(PICO_PLATFORM rp2350)
set(PICO_BOARD pico2)

# Include build functions from Pico SDK
include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(gbaflashcart_bootloader C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (NOT DEFINED BUILD_TIME_PROVIDED_VERSION)
    set(BUILD_TIME_PROVIDED_VERSION 255.255.255)
    if (NOT DEFINED BUILD_VERSION_TYPE)
      set(BUILD_VERSION_TYPE U)
      # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb -DDEBUG=1")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
    endif()
else ()
  if (NOT DEFINED BUILD_VERSION_TYPE)
    set(BUILD_VERSION_TYPE R)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
    endif()
endif()
message("BUILD_TIME_PROVIDED_VERSION=${BUILD_TIME_PROVIDED_VERSION} BUILD_VERSION_TYPE=${BUILD_VERSION_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_C_FLAGS}")

project(${PROJECT_NAME} VERSION ${BUILD_TIME_PROVIDED_VERSION})

configure_file(include/BuildVersion.h.in include/BuildVersion.h)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_subdirectory(libs/git-commit-tracking)


macro(xxd_embed FILE_KEY FILE_PATH SOURCES)
	# Translate content into comma-separated byte codes.
	set(FILE_HEX "${CMAKE_CURRENT_BINARY_DIR}/include/${FILE_KEY}.hex.h")
	add_custom_command(
		OUTPUT ${FILE_HEX}
		COMMAND xxd -c 16 -n ${FILE_KEY} -i ${FILE_PATH} > ${FILE_HEX} || cp ${CMAKE_CURRENT_SOURCE_DIR}/${FILE_KEY}.hex.h.bk ${FILE_HEX}
		COMMENT "Generating hex representation of ${FILE_PATH} file content"
		)
	set_source_files_properties("${FILE_HEX}" PROPERTIES GENERATED TRUE)
	# Submit the resulting source file for compilation
	list(APPEND ${SOURCES} ${FILE_HEX})
	# Disable optimization for this source file, as it would otherwise take too long
	# to compile, in case of big files.
	set_source_files_properties(${FILE_HEX} PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})
	message(STATUS "Source file ${FILE_HEX} shall be added to ${SOURCES}")
endmacro()

xxd_embed("bootloader_gba" ${CMAKE_CURRENT_SOURCE_DIR}/../gba/bootloader_mb.gba SRCS)


add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/gbabus.c
    src/DMAWriteWhatWhere.c
    ${CMAKE_CURRENT_BINARY_DIR}/include/bootloader_gba.hex.h
    ${SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/git-commit-tracking
  ${CMAKE_CURRENT_BINARY_DIR}/include    
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    cmake_git_commit_tracking
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/gbabus.pio)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_set_linker_script(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/linkerscript.ld)