CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -O0 -static $(INCLUDE_DIR)
LDFLAGS = -Lrplidar_sdk/output/Linux/Release
LDLIBS = -pthread -li2c -lrt -lpthread -lsl_lidar_sdk

#Get short git sha one from librairie-commune
PART1 = $(shell echo "-DGIT_COMMIT_SHA_PART1=0x$(shell echo $(shell git -C ../librairie-commune rev-parse --short HEAD) | cut -c1-2)")
PART2 = $(shell echo "-DGIT_COMMIT_SHA_PART2=0x$(shell echo $(shell git -C ../librairie-commune rev-parse --short HEAD) | cut -c3-4)")
PART3 = $(shell echo "-DGIT_COMMIT_SHA_PART3=0x$(shell echo $(shell git -C ../librairie-commune rev-parse --short HEAD) | cut -c5-6)")
PART4 = $(shell echo "-DGIT_COMMIT_SHA_PART4=0x$(shell echo $(shell git -C ../librairie-commune rev-parse --short HEAD) | cut -c7-8)")
CXXFLAGS += $(PART1) $(PART2) $(PART3) $(PART4)

INCLUDE_DIR = -Iinclude
INCLUDE_DIR += -Irplidar_sdk/sdk/include
INCLUDE_DIR += -Irplidar_sdk/sdk/src
INCLUDE_DIR += -I../librairie-commune/common/include
INCLUDE_DIR += -I../librairie-commune/master/include
INCLUDE_DIR += -I../cdfr2024-programme-Actionneur/include/common
INCLUDE_DIR += -IWiringPi/wiringPi

BINDIR = bin
TARGET = $(BINDIR)/programCDFR
TEST_TARGET = $(BINDIR)/tests

SRCDIR = src
SRCDIR_LIBCOM = ../librairie-commune/master/src
SRCDIR_TEST = tests

OBJDIR = obj
OBJDIR_MAIN = $(OBJDIR)/main_obj
OBJDIR_LIBCOM = $(OBJDIR)/lib_com_obj
OBJDIR_TEST = $(OBJDIR)/test_obj


# Recursively find .cpp files in SRCDIR and SRCDIR_LIBCOM
SRC = $(shell find $(SRCDIR) -name "*.cpp")
SRC_LIB_COM = $(shell find $(SRCDIR_LIBCOM) -name "*.cpp")

# Generate corresponding object files
OBJ = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR_MAIN)/%.o,$(SRC))
OBJ += $(patsubst $(SRCDIR_LIBCOM)/%.cpp,$(OBJDIR_LIBCOM)/%.o,$(SRC_LIB_COM))

SRC_NO_MAIN = $(filter-out $(SRCDIR)/main.cpp \
			$(SRCDIR)/restAPI/restAPI.cpp \
			$(SRCDIR)/navigation/navigation.cpp \
			$(SRCDIR)/actions/functions.cpp \
			$(SRCDIR)/actions/action.cpp \
			$(SRCDIR)/actions/revolver.cpp \
			$(SRCDIR)/lidar/Lidar.cpp \
			$(SRCDIR)/defs/tableState.cpp \
		, $(SRC) )
SRC_TEST = $(wildcard $(SRCDIR_TEST)/*.cpp)
OBJ_NO_MAIN = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR_MAIN)/%.o,$(SRC_NO_MAIN))
OBJ_NO_MAIN += $(patsubst $(SRCDIR_LIBCOM)/%.cpp,$(OBJDIR_LIBCOM)/%.o,$(SRC_LIB_COM))
TEST_OBJ = $(patsubst $(SRCDIR_TEST)/%.cpp,$(OBJDIR_TEST)/%.o,$(SRC_TEST))

DEPENDS := $(shell find obj -type f -name '*.d')


.PHONY: all clean tests clean-all deploy run deploy-install deploy-uninstall deploy-tests


all:
	$(MAKE) -j$(expr $(nproc) \- 2) check $(BINDIR) build_lidarLib $(TARGET) $(TEST_TARGET) copy_html copy_lidar copy_aruco
	@echo "Compilation terminée. Exécutez '(cd $(BINDIR) && sudo ./programCDFR)' pour exécuter le programme."

-include $(DEPENDS)

check:
	@if [ ! -d "$(SRCDIR_LIBCOM)" ]; then \
		echo "Error: The path SRCDIR_LIBCOM ($(SRCDIR_LIBCOM)) is incorrect or does not exist"; \
		echo "Don't work only in CDFR25. Clone CDFR :"; \
		echo "https://github.com/robotronik/CDFR"; \
		exit 1; \
	fi

$(TARGET): $(OBJ) | $(BINDIR)
	@echo "--------------------------------- Compilation du programme principal... ---------------------------------"
	@echo " APP  $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS) -Llib/x86_64-linux-gnu

$(TEST_TARGET): $(OBJ_NO_MAIN) $(TEST_OBJ) | $(BINDIR)
	@echo "--------------------------------- Compilation des tests... ---------------------------------"
	@echo " APP  $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS) -Llib/x86_64-linux-gnu

$(OBJDIR_MAIN)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR_MAIN)
	@echo " CXX  $@"
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR_LIBCOM)/%.o: $(SRCDIR_LIBCOM)/%.cpp | $(OBJDIR_LIBCOM)
	@mkdir -p $(dir $@)
	@echo " CXX  $@"
	@$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR_TEST)/%.o: $(SRCDIR_TEST)/%.cpp | $(OBJDIR_TEST)
	@mkdir -p $(dir $@)
	@echo " CXX  $@"
	@$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR_LIBCOM):
	@echo " DIR  $@"
	@mkdir -p $@

$(OBJDIR_MAIN):
	@echo " DIR  $@"
	@mkdir -p $@

$(OBJDIR_TEST):
	@echo " DIR  $@"
	@mkdir -p $@

$(BINDIR):
	@echo " DIR  $@"
	@mkdir -p $@

tests: build_lidarLib $(TEST_TARGET) copy_lidar
	@echo "--------------------------------- Exécution des tests... ---------------------------------"
	cd $(BINDIR) && ./tests

# Define the lidarLib target
build_lidarLib:
	@echo "--------------------------------- Compilation de lidarLib... ---------------------------------"
	$(MAKE) -C rplidar_sdk



# Cross-compiler and flags for Raspberry Pi (ARM architecture)
CROSS_COMPILE_PREFIX = aarch64-linux-gnu
ARM_CXX = $(CROSS_COMPILE_PREFIX)-g++

# Raspberry Pi Deployment Info
PI_USER = robotronik
PI_HOST = raspitronik.local
PI_DIR = /home/$(PI_USER)/CDFR2025

# Define the ARM target and object directory for cross-compilation
ARMBINDIR = arm_bin
OBJDIR_ARM = $(OBJDIR)/arm_obj
OBJDIR_ARM_LIBCOM = $(OBJDIR)/arm_obj_lib_com
OBJDIR_ARM_TEST = $(OBJDIR)/arm_test_obj

ARM_TARGET = $(ARMBINDIR)/programCDFR
ARM_TEST_TARGET = $(ARMBINDIR)/tests

ARM_OBJ = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR_ARM)/%.o,$(SRC))
ARM_OBJ += $(patsubst $(SRCDIR_LIBCOM)/%.cpp,$(OBJDIR_ARM_LIBCOM)/%.o,$(SRC_LIB_COM))

ARM_OBJ_NO_MAIN = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR_ARM)/%.o,$(SRC_NO_MAIN))
ARM_OBJ_NO_MAIN += $(patsubst $(SRCDIR_LIBCOM)/%.cpp,$(OBJDIR_ARM_LIBCOM)/%.o,$(SRC_LIB_COM))
ARM_TEST_OBJ = $(patsubst $(SRCDIR_TEST)/%.cpp,$(OBJDIR_ARM_TEST)/%.o,$(SRC_TEST))


# Compile all object files for ARM
$(OBJDIR_ARM)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR_ARM)
	@mkdir -p $(dir $@)
	@echo " ARM_CXX  $@"
	@$(ARM_CXX) $(CXXFLAGS) -D__CROSS_COMPILE_ARM__ -MMD -MP -c $< -o $@
$(OBJDIR_ARM):
	@echo " ARM_DIR  $@"
	@mkdir -p $@

$(OBJDIR_ARM_LIBCOM)/%.o: $(SRCDIR_LIBCOM)/%.cpp | $(OBJDIR_ARM_LIBCOM)
	@mkdir -p $(dir $@)
	@echo " ARM_CXX  $@"
	@$(ARM_CXX) $(CXXFLAGS) -D__CROSS_COMPILE_ARM__ -MMD -MP -c $< -o $@
$(OBJDIR_ARM_LIBCOM):
	@echo " ARM_DIR  $@"
	@mkdir -p $@

# Compile all test object files for ARM
$(OBJDIR_ARM_TEST)/%.o: $(SRCDIR_TEST)/%.cpp | $(OBJDIR_ARM_TEST)
	@mkdir -p $(dir $@)
	@echo " ARM_CXX  $@"
	@$(ARM_CXX) $(CXXFLAGS) -D__CROSS_COMPILE_ARM__ -MMD -MP -c $< -o $@
$(OBJDIR_ARM_TEST):
	@echo " ARM_DIR  $@"
	@mkdir -p $@


# Create the ARM binary directory
$(ARMBINDIR):
	@echo " ARM_DIR  $@"
	@mkdir -p $@

# Cross-compile and link for Raspberry Pi
$(ARM_TARGET): $(ARM_OBJ) | $(ARMBINDIR)
	@echo "--------------------------------- Compilation du programme principal... ---------------------------------"
	$(ARM_CXX) $(CXXFLAGS) -D__CROSS_COMPILE_ARM__ -o $@ $^ $(LDFLAGS) $(LDLIBS) -Llib/aarch64-linux-gnu

$(ARM_TEST_TARGET): $(ARM_OBJ_NO_MAIN) $(ARM_TEST_OBJ) | $(ARMBINDIR)
	@echo "--------------------------------- Compilation des tests... ---------------------------------"
	$(ARM_CXX) $(CXXFLAGS) -D__CROSS_COMPILE_ARM__ -o $@ $^ $(LDFLAGS) $(LDLIBS) -Llib/aarch64-linux-gnu

# Deploy target
deploy:
	$(MAKE) -j$(expr $(nproc) \- 2) check build_arm_lidarLib $(ARM_TARGET) copy_html_arm copy_install_sh copy_aruco_arm
	@echo "--------------------------------- Deploiement vers le Raspberry Pi... ---------------------------------"
	ssh $(PI_USER)@$(PI_HOST) 'mkdir -p $(PI_DIR)'
	rsync -av --progress ./$(ARMBINDIR) $(PI_USER)@$(PI_HOST):$(PI_DIR)

deploy-tests: check build_arm_lidarLib $(ARM_TEST_TARGET) copy_lidar_arm
	@echo "--------------------------------- Exécution des tests... ---------------------------------"
	ssh $(PI_USER)@$(PI_HOST) 'mkdir -p $(PI_DIR)'
	rsync -av --progress ./$(ARMBINDIR) $(PI_USER)@$(PI_HOST):$(PI_DIR)
	ssh $(PI_USER)@$(PI_HOST) '(cd $(PI_DIR)/$(ARMBINDIR) && ./tests)'

deploy-install: deploy copy_install_sh
	ssh $(PI_USER)@$(PI_HOST) 'mkdir -p $(PI_DIR)'
	rsync -av --progress ./$(ARMBINDIR) $(PI_USER)@$(PI_HOST):$(PI_DIR)
	ssh $(PI_USER)@$(PI_HOST) '(cd $(PI_DIR)/$(ARMBINDIR) && sudo ./autoRunInstaller.sh --install programCDFR)'

deploy-uninstall: copy_install_sh
	ssh $(PI_USER)@$(PI_HOST) 'mkdir -p $(PI_DIR)'
	rsync -av --progress ./$(ARMBINDIR) $(PI_USER)@$(PI_HOST):$(PI_DIR)
	ssh $(PI_USER)@$(PI_HOST) '(cd $(PI_DIR)/$(ARMBINDIR) && sudo ./autoRunInstaller.sh --uninstall programCDFR)'

run: deploy
	ssh $(PI_USER)@$(PI_HOST) '$(PI_DIR)/$(ARM_TARGET)'

# Define the lidarLib target
build_arm_lidarLib:
	@echo "--------------------------------- Compilation de lidarLib pour ARM64... ---------------------------------"
	@cd rplidar_sdk && \
	chmod +x ./cross_compile.sh && \
	CROSS_COMPILE_PREFIX=$(CROSS_COMPILE_PREFIX) ./cross_compile.sh



# Rule to copy the HTML directory to bin
copy_html: | $(BINDIR)
	cp -r html $(BINDIR)
# Rule to copy the HTML directory to the arm bin
copy_html_arm: | $(ARMBINDIR)
	cp -r html $(ARMBINDIR)

# Rule to copy the lidar json directory to the bin
copy_lidar: | $(BINDIR)
	cp -r tests/lidar $(BINDIR)
# Rule to copy the lidar json directory to the arm bin
copy_lidar_arm: | $(ARMBINDIR)
	cp -r tests/lidar $(ARMBINDIR)
# Rule to copy the autoRunInstaller.sh file to the arm bin directory
copy_install_sh: | $(ARMBINDIR)
	cp autoRunInstaller.sh $(ARMBINDIR)

# Rule to copy the python aruco detection directory to the bin
copy_aruco: | $(BINDIR)
	cp -r ../PythonArucoOpenCV/data $(BINDIR)
	cp ../PythonArucoOpenCV/detect_aruco.py $(BINDIR)
# Rule to copy the python aruco detection directory to the arm bin
copy_aruco_arm: | $(ARMBINDIR)
	cp -r ../PythonArucoOpenCV/data $(ARMBINDIR)
	cp ../PythonArucoOpenCV/detect_aruco.py $(ARMBINDIR)



clean:
	@echo "--------------------------------- Nettoyage... ---------------------------------"
	rm -rf $(OBJDIR) $(BINDIR) $(ARMBINDIR) $(OBJDIR_MAIN) $(OBJDIR_LIBCOM) $(OBJDIR_TEST) $(OBJDIR_ARM) $(OBJDIR_ARM_MAIN) $(OBJDIR_ARM_LIBCOM) $(OBJDIR_ARM_TEST)

# Clean lidarLib specifically
clean-lidarLib:
	@echo "--------------------------------- Nettoyage de lidarLib... ---------------------------------"
	$(MAKE) -C rplidar_sdk clean

# Combined clean for all
clean-all: clean clean-lidarLib