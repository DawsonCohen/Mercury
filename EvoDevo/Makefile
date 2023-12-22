ifndef config
  config=release
endif

.PHONY: all clean vars

CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O3

CU := nvcc
CUFLAGS := -gencode=arch=compute_75,code=sm_75 -gencode=arch=compute_61,code=sm_61 -O3 -use_fast_math

# Debug build flags
ifeq ($(config),debug)
	CUFLAGS += -pg -g -G

	CUDEBUG := -g -G -lineinfo -Xptxas -dlcm=cg
	DEBUG := -g
endif


ifeq ($(BUILD_LOCATION),local)
	CUFLAGS += -Xcudafe --diag_suppress=20050
	CUFLAGS += -Xcudafe --diag_suppress=20015
	CUFLAGS += -Xcudafe --diag_suppress=20013
	CUFLAGS += -Xcudafe --diag_suppress=20012
endif

SRCDIR := src
OBJDIR := obj
BINDIR := .

# Source directories
CXX_SRCDIRS := EvoDevo/Simulator EvoDevo/Evolvables EvoDevo/Optimizer EvoDevo/Util EvoDevo/Core
CUDA_SRCDIRS := EvoDevo/Simulator EvoDevo/Evolvables

MAIN_SRC := main.cpp
EXECUTABLE = $(BINDIR)/evodevo
# Define target executable
ifdef BENCHMARK
MAIN_SRC := benchmark.cpp
EXECUTABLE = $(BINDIR)/benchmark
else ifdef TESTS
MAIN_SRC := test.cpp
CXX_SRCDIRS += EvoDevo/Tests
EXECUTABLE = $(BINDIR)/runtests
endif

PCH = $(SRCDIR)/evpch.h

INCLUDE_ALL := $(foreach dir,$(CXX_SRCDIRS),-I$(wildcard $(SRCDIR)/$(dir)))

# C++ and CUDA source files
CXX_SRCS := $(foreach dir,$(CXX_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cpp))
CUDA_SRCS := $(foreach dir,$(CUDA_SRCDIRS),$(wildcard $(SRCDIR)/$(dir)/*.cu))

# Object files
CXX_OBJS := $(patsubst %.cpp,$(OBJDIR)/%.o,$(notdir $(CXX_SRCS)))
CUDA_OBJS := $(patsubst %.cu,$(OBJDIR)/%.o,$(notdir $(CUDA_SRCS)))

# Included libraries
INCLUDE_DIRS := -Isrc -Ivendor/spdlog/include -Ivendor/gmp -Ivendor/CGAL-5.6/include -Ivendor/eigen
LIB_DIRS := -Lvendor/lib
LIB_NAMES := -lgmp

CXXFLAGS += $(INCLUDE_DIRS)
CUFLAGS += $(INCLUDE_DIRS)

all: $(EXECUTABLE)

$(EXECUTABLE): $(CXX_OBJS) $(CUDA_OBJS) $(SRCDIR)/$(MAIN_SRC)
	$(CU) $(CUFLAGS) $(LIB_DIRS) $(INCLUDE_ALL) $^ -o $@ $(LIB_NAMES)

ifdef TESTS
# Tests
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Tests/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDE_ALL) -c -o $@ $<
endif

# Simulator
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Simulator/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Simulator/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -c -o $@ $<

# Optimizer
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Optimizer/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Evolvables/%.cu | $(OBJDIR)
	$(CU) $(CUFLAGS) -c -o $@ $<

# Evolvables
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Evolvables/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Util
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Util/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Core
$(OBJDIR)/%.o: $(SRCDIR)/EvoDevo/Core/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

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
	$(info PCH is $(PCH))

	$(info CXXFLAGS is $(CXXFLAGS))
	$(info CUFLAGS is $(CUFLAGS))

clean:
	rm -rf $(OBJDIR)/* $(EXECUTABLE)
