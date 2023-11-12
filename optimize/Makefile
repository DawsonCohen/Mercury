
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3 -pg

CU := nvcc
CUFLAGS := -gencode=arch=compute_75,code=sm_75 -gencode=arch=compute_61,code=sm_61 -O3 -pg -use_fast_math -maxrregcount=32

# CUPROF := -pg
# PROF := -pg

CUDEBUG := -g -G -lineinfo -Xptxas -dlcm=cg
DEBUG := -g

ifeq ($(BUILD_LOCATION),local)
	CUFLAGS += -Xcudafe --diag_suppress=20050
	CUFLAGS += -Xcudafe --diag_suppress=20015
	CUFLAGS += -Xcudafe --diag_suppress=20013
	CUFLAGS += -Xcudafe --diag_suppress=20012
endif

SRCDIR := ..
OBJDIR := obj
BINDIR := .

# Source directories
CXX_SRCDIRS := common/simulator common/evolvables optimize/optimizer optimize/optimizer/util common/util
CUDA_SRCDIRS := common/simulator common/evolvables

MAIN_SRC := main_optimize.cpp
EXECUTABLE = $(BINDIR)/evodevo
# Define target executable
ifdef BENCHMARK
MAIN_SRC := benchmark.cpp
EXECUTABLE = $(BINDIR)/benchmark
else ifdef TESTS
MAIN_SRC := test_optimize.cpp
CXX_SRCDIRS += optimize/tests
EXECUTABLE = $(BINDIR)/runtests
endif

INCLUDE_ALL := $(foreach dir,$(CXX_SRCDIRS),-I$(wildcard $(SRCDIR)/$(dir)))

# C++ and CUDA source files
CXX_SRCS := $(foreach dir,$(CXX_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cpp))
CUDA_SRCS := $(foreach dir,$(CUDA_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cu))

# Object files
CXX_OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(notdir $(CXX_SRCS)))
CUDA_OBJS := $(patsubst %.cu,$(OBJDIR)/%.o,$(notdir $(CUDA_SRCS)))

# Included libraries
EIGEN_LIB_DIR := $(SRCDIR)/common/lib
EIGEN_LIB_NAME := cudart

INCLUDE_DIRS := -I$(EIGEN_LIB_DIR) -I$(SRCDIR)/common/util -I$(SRCDIR)/optimize/optimizer/util
LIB_DIRS := -L$(EIGEN_LIB_DIR)
LIB_NAMES := -l$(EIGEN_LIB_NAME)

CXXFLAGS += $(INCLUDE_DIRS)
CUFLAGS += $(INCLUDE_DIRS)

.PHONY: all clean vars debug

all: $(EXECUTABLE)

# Change this line to select a different main source file

$(EXECUTABLE): $(CXX_OBJS) $(CUDA_OBJS) $(SRCDIR)/optimize/$(MAIN_SRC)
	$(CU) $(CUFLAGS) $(LIB_DIRS) $(INCLUDE_ALL) $^ -o $@ $(LIB_NAMES)

ifdef TESTS
# Tests
$(OBJDIR)/%.o: $(SRCDIR)/optimize/tests/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDE_ALL) -c -o $@ $<
endif

# Simulator
$(OBJDIR)/%.o: $(SRCDIR)/common/simulator/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(SRCDIR)/common/simulator/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -c -o $@ $<

# Optimizer
$(OBJDIR)/%.o: $(SRCDIR)/optimize/optimizer/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Optimizer Util
$(OBJDIR)/%.o: $(SRCDIR)/optimize/optimizer/util/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/common/evolvables/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/common/simulator -I$(SRCDIR)/optimize/optimizer -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/common/evolvables/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -I$(SRCDIR)/common/simulator -I$(SRCDIR)/optimize/optimizer -c -o $@ $<

# Util
$(OBJDIR)/%.o: $(SRCDIR)/common/util/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/common/evolvables -I$(SRCDIR)/common/simulator -I$(SRCDIR)/common/optimizer -c -o $@ $<

$(OBJDIR):
	mkdir -p $@

# Target: Debug
debug: CXXFLAGS += $(DEBUG)
debug: CUFLAGS += $(CUDEBUG)
debug: clean all

vars:
	$(info CXX_SRCS is $(CXX_SRCS))
	$(info CUDA_SRCS is $(CUDA_SRCS))
	$(info CXX_OBJS is $(CXX_OBJS))
	$(info CUDA_OBJS is $(CUDA_OBJS))

	$(info INCLUDE_DIRS is $(INCLUDE_DIRS))
	$(info OPTIMIZE is $(OPTIMIZE))
	$(info BUILD_LOCATION is $(BUILD_LOCATION))

	$(info CXXFLAGS is $(CXXFLAGS))
	$(info CUFLAGS is $(CUFLAGS))

clean:
	rm -rf $(OBJDIR)/* $(EXECUTABLE)
