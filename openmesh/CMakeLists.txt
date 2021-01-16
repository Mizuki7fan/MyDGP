cmake_minimum_required(VERSION 3.5)

project(MyDGP)

#设置不生成MinSizeRel;RelWithDebInfo
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
add_definitions(/D_USE_MATH_DEFINES)

#path
SET(EIGEN_PATH "D:/LIBRARY/Eigen")
SET(OPENMESH_PATH "D:/LIBRARY/OpenMesh/src")
SET(OPENMESH_LIB_PATH "D:/LIBRARY/OpenMesh/build_vs/Build/lib")

#为所有项目都设置包含的库的位置
include_directories(${EIGEN_PATH})
include_directories(${OPENMESH_PATH})
link_directories(${OPENMESH_LIB_PATH})
link_libraries(
    debug OpenMeshToolsd 
    debug OpenMeshCored
    optimized OpenMeshTools 
    optimized OpenMeshCore
)

############################################################
# Project Homework 0 == Openmesh基本操作
############################################################

file (GLOB HW0_FILE
    src/hw0/main.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW0 ${HW0_FILE})

############################################################
# Project Homework 1 == Dijkstra和Minimal Spanning Tree
############################################################

############################################################
# Project Homework 2 == 离散微分几何
############################################################

############################################################
# Project Homework 3.1 == Fairing
############################################################

############################################################
# Project Homework 3.1 == Laplacian Smoothing
############################################################

############################################################
# Project Homework 3.1 == Bilateral Mesh Denoising
############################################################

############################################################
# Project Homework 3.1 == Bilateral Normal Filtering
############################################################

############################################################
# Project Homework 4.1 == Tutte Parameterization
############################################################

file (GLOB HW41_FILE
    src/hw4/tutte.cpp
    src/general/MeshDefinition.*
)
add_executable(${PROJECT_NAME}_HW41 ${HW41_FILE})

############################################################
# Project Homework 4.2 == Lscm
############################################################

file (GLOB HW42_FILE
    src/general/MeshDefinition.*
    src/general/MeshUtilities.*
    src/hw4/lscm.cpp
)
add_executable(${PROJECT_NAME}_HW42 ${HW42_FILE})

############################################################
# Project Homework 4.3 == ABF
############################################################

file (GLOB HW43_FILE
    src/general/MeshDefinition.*
    src/general/MeshUtilities.*
    src/hw4/abf.cpp
)
add_executable(${PROJECT_NAME}_HW43 ${HW43_FILE})