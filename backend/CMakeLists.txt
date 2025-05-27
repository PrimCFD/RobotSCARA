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

# Local include paths
set(LOCAL_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/nlohmann
    ${CMAKE_SOURCE_DIR}/include/Eigen
    ${CMAKE_SOURCE_DIR}/include/boost
)

# qpOASES
set(QPOASES_DIR ${CMAKE_SOURCE_DIR}/external/qpOASES)
include_directories(${QPOASES_DIR}/include)

set(QPOASES_SRC
    ${QPOASES_DIR}/src/BLASReplacement.cpp
    ${QPOASES_DIR}/src/Bounds.cpp
    ${QPOASES_DIR}/src/Constraints.cpp
    ${QPOASES_DIR}/src/Flipper.cpp
    ${QPOASES_DIR}/src/Indexlist.cpp
    ${QPOASES_DIR}/src/LAPACKReplacement.cpp
    ${QPOASES_DIR}/src/Matrices.cpp
    ${QPOASES_DIR}/src/MessageHandling.cpp
    ${QPOASES_DIR}/src/Options.cpp
    ${QPOASES_DIR}/src/OQPinterface.cpp
    ${QPOASES_DIR}/src/QProblem.cpp
    ${QPOASES_DIR}/src/QProblemB.cpp
    ${QPOASES_DIR}/src/SolutionAnalysis.cpp
    ${QPOASES_DIR}/src/SparseSolver.cpp
    ${QPOASES_DIR}/src/SQProblem.cpp
    ${QPOASES_DIR}/src/SQProblemSchur.cpp
    ${QPOASES_DIR}/src/SubjectTo.cpp
    ${QPOASES_DIR}/src/Utils.cpp
)

add_library(qpOASES STATIC ${QPOASES_SRC})
target_include_directories(qpOASES PUBLIC ${QPOASES_DIR}/include)

# Main executable
add_executable(backend
    src/main_server.cpp
    src/RobotModel.cpp
    src/SIL.cpp
    src/TorqueQP.cpp
)

target_include_directories(backend PRIVATE ${LOCAL_INCLUDES})

target_compile_definitions(backend PRIVATE 
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
    BOOST_ALL_NO_LIB
)

# Link QP solver
target_link_libraries(backend PRIVATE qpOASES)
