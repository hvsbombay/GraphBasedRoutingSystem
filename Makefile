# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I.
LDFLAGS = 

# Directories
PHASE1_DIR = Phase-1
PHASE2_DIR = Phase-2
PHASE3_DIR = Phase-3

# Phase 1
PHASE1_SRC = $(PHASE1_DIR)/main.cpp $(PHASE1_DIR)/ShortestPath.cpp $(PHASE1_DIR)/KNN.cpp $(PHASE1_DIR)/Graph.cpp
PHASE1_OBJ = $(PHASE1_SRC:.cpp=.o)

# Phase 2
PHASE2_SRC = $(PHASE2_DIR)/main.cpp $(PHASE2_DIR)/KShortestPaths.cpp $(PHASE2_DIR)/ApproxShortestPath.cpp $(PHASE2_DIR)/Graph.cpp
PHASE2_OBJ = $(PHASE2_SRC:.cpp=.o)

# Phase 3
PHASE3_SRC = $(PHASE3_DIR)/main.cpp $(PHASE3_DIR)/Graph.cpp $(PHASE3_DIR)/DeliveryScheduler.cpp
PHASE3_OBJ = $(PHASE3_SRC:.cpp=.o)

# Targets
.PHONY: all clean phase1 phase2 phase3

all: phase1 phase2 phase3

phase1: $(PHASE1_OBJ)
	$(CXX) $(CXXFLAGS) -o phase1 $(PHASE1_OBJ) $(LDFLAGS)
	@echo "Phase 1 executable created successfully!"

phase2: $(PHASE2_OBJ)
	$(CXX) $(CXXFLAGS) -o phase2 $(PHASE2_OBJ) $(LDFLAGS)
	@echo "Phase 2 executable created successfully!"

phase3: $(PHASE3_OBJ)
	$(CXX) $(CXXFLAGS) -o phase3 $(PHASE3_OBJ) $(LDFLAGS)
	@echo "Phase 3 executable created successfully!"

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	@if exist phase1.exe del phase1.exe
	@if exist phase2.exe del phase2.exe
	@if exist phase3.exe del phase3.exe
	@if exist Phase-1\*.o del Phase-1\*.o
	@if exist Phase-2\*.o del Phase-2\*.o
	@if exist Phase-3\*.o del Phase-3\*.o
	@echo Cleaned all build artifacts!

# Individual clean targets
clean-phase1:
	@if exist phase1.exe del phase1.exe
	@if exist Phase-1\*.o del Phase-1\*.o

clean-phase2:
	@if exist phase2.exe del phase2.exe
	@if exist Phase-2\*.o del Phase-2\*.o

clean-phase3:
	@if exist phase3.exe del phase3.exe
	@if exist Phase-3\*.o del Phase-3\*.o
