
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3

CU := nvcc
CUFLAGS := -gencode=arch=compute_75,code=sm_75 -gencode=arch=compute_61,code=sm_61 -O3

ifeq ($(BUILD_LOCATION),local)
	CUFLAGS += -Xcudafe --diag_suppress=20050
	CUFLAGS += -Xcudafe --diag_suppress=20015
	CUFLAGS += -Xcudafe --diag_suppress=20013
	CUFLAGS += -Xcudafe --diag_suppress=20012
endif


CXXFLAGS += -DOPTIMIZE
CUFLAGS += -DOPTIMIZE

SRCDIR := .
OBJDIR := obj
BINDIR := .

# Source directories
CXX_SRCDIRS := simulator evolvables optimizer util
CUDA_SRCDIRS := simulator evolvables

INCLUDE_ALL := $(foreach dir,$(CXX_SRCDIRS),-I$(wildcard $(SRCDIR)/$(dir)))

# C++ and CUDA source files
CXX_SRCS := $(foreach dir,$(CXX_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cpp))
CUDA_SRCS := $(foreach dir,$(CUDA_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cu))

# Object files
CXX_OBJS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(CXX_SRCS))
CUDA_OBJS := $(patsubst $(SRCDIR)/%.cu,$(OBJDIR)/%.o,$(CUDA_SRCS))

# Included libraries
EIGEN_LIB_DIR := $(SRCDIR)/lib
EIGEN_LIB_NAME := cudart

INCLUDE_DIRS := -I$(EIGEN_LIB_DIR)
LIB_DIRS := -L$(EIGEN_LIB_DIR)
LIB_NAMES := -l$(EIGEN_LIB_NAME)

CXXFLAGS += $(INCLUDE_DIRS)
CUFLAGS += $(INCLUDE_DIRS)

# Define target executable
EXECUTABLE = $(BINDIR)/run_evo

.PHONY: all clean vars

all: $(EXECUTABLE)

# Change this line to select a different main source file
MAIN_SRC := main.cpp

$(EXECUTABLE): $(CXX_OBJS) $(CUDA_OBJS) $(SRCDIR)/$(MAIN_SRC)
	$(CU) $(CUFLAGS) $(LIB_DIRS) $(INCLUDE_ALL) $^ -o $@ $(LIB_NAMES)

# Simulator
$(OBJDIR)/simulator/%.o: $(SRCDIR)/simulator/%.cpp | $(OBJDIR)/simulator
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/simulator/%.o: $(SRCDIR)/simulator/%.cu | $(OBJDIR)/simulator
	$(CU) $(CUFLAGS) -c -o $@ $<

# Optimizer
$(OBJDIR)/optimizer/%.o: $(SRCDIR)/optimizer/%.cpp | $(OBJDIR)/optimizer
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Evolvables
$(OBJDIR)/evolvables/%.o: $(SRCDIR)/evolvables/%.cpp | $(OBJDIR)/evolvables
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/simulator -I$(SRCDIR)/optimizer -c -o $@ $<

# Evolvables
$(OBJDIR)/evolvables/%.o: $(SRCDIR)/evolvables/%.cu | $(OBJDIR)/evolvables
	$(CU) $(CUFLAGS) -I$(SRCDIR)/simulator -I$(SRCDIR)/optimizer -c -o $@ $<

# Util
$(OBJDIR)/util/%.o: $(SRCDIR)/util/%.cpp | $(OBJDIR)/util
	$(CXX) $(CXXFLAGS) -I$(SRCDIR)/evolvables -I$(SRCDIR)/simulator -I$(SRCDIR)/optimizer -c -o $@ $<

$(OBJDIR)/simulator:
	mkdir -p $@

$(OBJDIR)/optimizer:
	mkdir -p $@

$(OBJDIR)/evolvables:
	mkdir -p $@

$(OBJDIR)/util:
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
