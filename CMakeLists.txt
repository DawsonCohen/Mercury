cmake_minimum_required( VERSION 3.1 )

project( phys C CXX CUDA )
# set( CMAKE_CXX_COMPILER /usr/bin/g++ )
# set( CMAKE_CUDA_COMPILER /usr/local/cuda-12.0/bin/nvcc )
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CUDA_STANDARD 17 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pg -W -Wall" )
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --ptxas-options=-v -ftz=true -prec-div=false -prec-sqrt=false --expt-relaxed-constexpr")

if(DEFINED ENV{BUILD_LOCATION} AND "$ENV{BUILD_LOCATION}" STREQUAL "local")
	set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20012")
endif()

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.0)
find_package( CUDA REQUIRED )

## MAIN OPTIONS ##
add_definitions("-DOPTIMIZE")
# add_definitions("-DOPTIMIZE -DVISUALIZE")
# add_definitions("-DVIDEO -DVISUALIZE")

# add_definitions("-DOPTIMIZE -DWRITE_VIDEO")
# add_definitions("-DVERIFY -DVISUALIZE")
# add_definitions("-DVERIFY -DWRITE_VIDEO")

#[[
SIMULATOR OPTIONS
STRESS_COUNT - calculates each time a spring is the top/bottom stressed spring in a thread
FULL_STRESS - calculates exact stress in each spring over the simulation lifetime
]]#
# add_definitions("-DSTRESS_COUNT")
# add_definitions("-DSTRESS_COUNT -DFULL_STRESS -DWRITE_STRESS")

set( PHYS
	# benchmark.cpp
        main.cpp

        simulator/Simulator.cu

        evolvables/SoftBody.cpp
        evolvables/VoxelRobot.cpp

        util/util.cpp
)

set( RENDER
        renderer/src/Camera.cpp
        renderer/src/Shader.cpp
        renderer/src/Mesh.cpp
        renderer/src/Renderer.cpp
        renderer/src/VAO.cpp
        renderer/src/VBO.cpp
        renderer/src/EBO.cpp
        renderer/src/Model.cpp

        models/plane_model.cpp
)

if(DEFINED ENV{BUILD_LOCATION} AND "$ENV{BUILD_LOCATION}" STREQUAL "remote")
  add_executable( phys
    ${PHYS}
  )
else()
  add_executable( phys
    ${PHYS} ${RENDER}
  )
endif()
  
  
set_target_properties( phys PROPERTIES CUDA_ARCHITECTURES "61;75" )

target_include_directories(phys PUBLIC
	lib
	simulator
	optimizer
	util
	evolvables)
		
if(DEFINED ENV{BUILD_LOCATION} AND "$ENV{BUILD_LOCATION}" STREQUAL "local")
# find_package( OpenCV REQUIRED )
add_subdirectory( renderer )

target_include_directories(phys PUBLIC
		renderer/src
		models
		)
		
target_link_libraries( phys 
	PUBLIC
		glad
		glfw
		glm
		stb
		${OpenCV_LIBS}
	)
endif()

# target_compile_options(phys PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
# 					-O3
# 					-Xcudafe --diag_suppress=20012
# 					--ptxas-options=-v
# 					-ftz=true
# 					-prec-div=false
# 					-prec-sqrt=false
# 					-Wno-unknown-pragmas
# 					--expt-relaxed-constexpr
# 					>)
