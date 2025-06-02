cmake_minimum_required(VERSION 3.5)
project(backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output binaries to ../bin/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_SOURCE_DIR}/../bin)

# Configure parameter generation
set(PARAM_GEN_SCRIPT ${CMAKE_SOURCE_DIR}/../utils/generate_params.py)
set(INPUT_JSON ${CMAKE_SOURCE_DIR}/../configs/temp.json)
set(GENERATED_HEADER ${CMAKE_SOURCE_DIR}/include/HardcodedParams.hpp)  # Output to backend/include

# Determine Python executable path based on platform
if(WIN32)
    set(VENV_PYTHON ${CMAKE_SOURCE_DIR}/../venv/Scripts/python.exe)
else()
    set(VENV_PYTHON ${CMAKE_SOURCE_DIR}/../venv/bin/python)
endif()

# Create custom command to generate parameters header in backend/include
add_custom_command(
    OUTPUT ${GENERATED_HEADER}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/include  # Ensure dir exists
    COMMAND ${VENV_PYTHON} ${PARAM_GEN_SCRIPT} ${INPUT_JSON} ${CMAKE_SOURCE_DIR}/include
    DEPENDS ${PARAM_GEN_SCRIPT} ${INPUT_JSON}
    COMMENT "Generating hardcoded parameters in include/ using: ${VENV_PYTHON}"
    VERBATIM
)

# Local include paths
set(LOCAL_INCLUDES
    ${CMAKE_SOURCE_DIR}/include  # This is backend/include
)

# Main executable
add_executable(backend
    src/main_server.cpp
    src/Controller.cpp
    src/RobotModel.cpp
    src/SIL.cpp
)

# Create custom target for parameter generation
add_custom_target(generate_params DEPENDS ${GENERATED_HEADER})
add_dependencies(backend generate_params)

# Include directories
target_include_directories(backend PRIVATE 
    ${LOCAL_INCLUDES}
)

# Platform-specific configurations
if(WIN32)
    target_compile_definitions(backend PRIVATE 
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
    )
    target_link_libraries(backend PRIVATE ws2_32)
else()
    # Linux-specific settings
    target_link_libraries(backend PRIVATE pthread)
endif()

# Set common compile definitions
target_compile_definitions(backend PRIVATE 
    EIGEN_NO_MALLOC
)