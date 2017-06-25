# Common library to build firmware targets against (so we don't need to keep re-adding the mbed lib)

# set the arm toolchain and include it so we can use it
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

file(GLOB_RECURSE mbed-common_SRC "main.c")

# append all of the accessory library include directories
list(APPEND mbed-common_INCLUDE_DIRS ${MBED_ASSEC_LIBS_INCLUDES})

# build the 'mbed-common' library that contains all the MBED stuff needed for
# both the base station and robot firmware
add_library(mbed-common ${mbed-common_SRC})

# depends on the mbed libraries and all of the accessory libraries
add_dependencies(mbed-common mbed_libraries ${MBED_ASSEC_LIBS_DEPENDS})

target_link_libraries(mbed-common ${MBED_LIBRARY})

target_include_directories(mbed-common PUBLIC ${mbed-common_INCLUDE_DIRS})

# only build firmware library if required
set_target_properties(mbed-common PROPERTIES EXCLUDE_FROM_ALL TRUE)
