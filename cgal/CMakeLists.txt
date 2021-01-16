cmake_minimum_required(VERSION 3.5)

project(MyDGP)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
if(CGAL_Qt5_FOUND)
  #required to use basic_viewer
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

############################################################
# Project Homework 0 == 学习使用CGAL
############################################################
#CGAL_Qt5 is needed for the drawing.

file (GLOB HW0_FILE
    src/hw0/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW0 ${HW0_FILE})
# target_include_directories(${PROJECT_NAME}_HW1
#     PRIVATE 
#         ${PROJECT_SOURCE_DIR}/include
# )
if(CGAL_Qt5_FOUND )
    target_link_libraries(${PROJECT_NAME}_HW0
        PUBLIC 
            CGAL::CGAL_Qt5
)
endif()

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
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW41 ${HW41_FILE})

if(CGAL_Qt5_FOUND )
    target_link_libraries(${PROJECT_NAME}_HW41
        PUBLIC 
            CGAL::CGAL_Qt5
)
endif()

############################################################
# Project Homework 3.1 == LSCM
############################################################

############################################################
# Project Homework 3.1 == ABF
############################################################