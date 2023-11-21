
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3

CU := nvcc
CUFLAGS := -gencode=arch=compute_75,code=sm_75 -gencode=arch=compute_61,code=sm_61 -O3

# Debug build flags
ifeq ($(dbg),1)
      CUFLAGS += -pg -lineinfo -pg -G
      BUILD_TYPE := debug
else
      BUILD_TYPE := release
endif

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
CXX_SRCDIRS := common/simulator common/evolvables optimize/optimizer common/util common/tests
CUDA_SRCDIRS := common/simulator common/evolvables

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

INCLUDE_DIRS := -I$(EIGEN_LIB_DIR) -I$(SRCDIR)/common/util
LIB_DIRS := -L$(EIGEN_LIB_DIR) 
LIB_NAMES := -l$(EIGEN_LIB_NAME) -lgmp

CXXFLAGS += $(INCLUDE_DIRS)
CUFLAGS += $(INCLUDE_DIRS)

# Define target executable
MAIN_SRC := test_common.cpp
EXECUTABLE = $(BINDIR)/runtests

.PHONY: all clean vars

all: $(EXECUTABLE)

# Change this line to select a different main source file

$(EXECUTABLE): $(CXX_OBJS) $(CUDA_OBJS) $(SRCDIR)/common/$(MAIN_SRC)
	$(CU) $(CUFLAGS) $(LIB_DIRS) $(INCLUDE_ALL) $^ -o $@ $(LIB_NAMES)

# Tests
$(OBJDIR)/%.o: $(SRCDIR)/common/tests/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDE_ALL) -c -o $@ $<

# Simulator
$(OBJDIR)/%.o: $(SRCDIR)/common/simulator/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(SRCDIR)/common/simulator/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -c -o $@ $<

# Optimizer
$(OBJDIR)/%.o: $(SRCDIR)/optimize/optimizer/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/common/evolvables/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/common/simulator -I$(SRCDIR)/optimize/optimizer -I/usr/include -lCGAL -lgmp -LCGAL_Core -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/common/evolvables/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -I$(SRCDIR)/common/simulator -I$(SRCDIR)/optimize/optimizer -c -o $@ $<

# Util
$(OBJDIR)/%.o: $(SRCDIR)/common/util/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/common/evolvables -I$(SRCDIR)/common/simulator -I$(SRCDIR)/common/optimizer -c -o $@ $<

$(OBJDIR):
	mkdir -p $@

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
