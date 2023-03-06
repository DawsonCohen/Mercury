cmake_minimum_required( VERSION 3.1 )

project( phys C CXX CUDA )
set( CMAKE_CXX_STANDARD 20 )
set( CMAKE_CUDA_STANDARD 20 )

include(FindCUDA/select_compute_arch)

CUDA_DETECT_INSTALLED_GPUS(INSTALLED_GPU_CCS_1)
string(STRIP "${INSTALLED_GPU_CCS_1}" INSTALLED_GPU_CCS_2)
string(REPLACE " " ";" INSTALLED_GPU_CCS_3 "${INSTALLED_GPU_CCS_2}")
string(REPLACE "." "" CUDA_ARCH_LIST "${INSTALLED_GPU_CCS_3}")
SET(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_LIST})

# find_package( OpenCV REQUIRED )

add_subdirectory( renderer )

set( PHYS
	# benchmark.cpp
	main.cpp

	renderer/src/Camera.cpp
	renderer/src/Shader.cpp
	renderer/src/Mesh.cpp
	renderer/src/Renderer.cpp
	renderer/src/VAO.cpp
	renderer/src/VBO.cpp
	renderer/src/EBO.cpp
	renderer/src/Model.cpp
	
	simulator/Simulator.cu
	
	evolvables/SoftBody.cpp
	evolvables/VoxelRobot.cpp

	models/plane.cpp
	models/robot.cpp

	optimizer/optimizer.cpp
	optimizer/Evaluator.cpp
)

add_definitions(-Wall -Wextra -O3 -pg)

# add_definitions("-DOPTIMIZE -DVISUALIZE")
add_definitions("-DVISUALIZE")

# add_definitions("-DOPTIMIZE -DWRITE_VIDEO")
# add_definitions("-DVERIFY -DVISUALIZE")
# add_definitions("-DVERIFY -DWRITE_VIDEO")

add_executable( phys 
${PHYS} 
)

target_include_directories(phys PUBLIC renderer/src)
target_include_directories(phys PUBLIC simulator)
target_include_directories(phys PUBLIC optimizer)
target_include_directories(phys PUBLIC util)
target_include_directories(phys PUBLIC models)
target_include_directories(phys PUBLIC evolvables)

target_link_libraries( phys 
	PUBLIC
		glad
		glfw
		glm
		stb
		${OpenCV_LIBS}
	)

target_compile_options(phys PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
					-Xcudafe --diag_suppress=20012
					>)