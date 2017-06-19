MAKE_FLAGS = --no-print-directory

# build a specified target with CMake and Ninja
# usage: $(call cmake_build_target, target, extraCmakeFlags)
define cmake_build_target
	mkdir -p build
	cd build && cmake -GNinja -Wno-dev --target $1 $2 .. && ninja $1
endef

# Similar to the above build target command, but for firmware.  This is used
# because CMake can only handle one toolchain at a time, so we build the MBED-
# targeted code separately.
define cmake_build_target_fw
	mkdir -p build/firmware
	cd build/firmware && cmake -Wno-dev --target $1 $2 ../.. && make $1 $(MAKE_FLAGS) -j
endef

all: robot

# Run both C++ and python unit tests
clean:
	cd build && ninja clean || true
	rm -rf build

# the alias names that point to the current set of firmware targets
robot: robot2015
robot-prog: robot2015-prog
fpga: fpga2015
fpga-prog: fpga2015-prog
kicker: kicker2015
kicker-prog: kicker2015-prog
firmware: firmware2015

fpga2015-test:
	$(call cmake_build_target_fw, fpga2015_iverilog)
fpga2015-test-strict:
	$(call cmake_build_target_fw, fpga2015_iverilog_strict)

# robot 2015 firmware
robot2015:
	$(call cmake_build_target_fw, robot2015)
robot2015-prog:
	$(call cmake_build_target_fw, robot2015-prog)

GDB_PORT ?= 3333
.INTERMEDIATE: build/robot2015-gdb.pid
build/robot2015-gdb.pid:
# this will cache sudo use without a password in the environment
# so we won't enter the gdb server and skip past the password prompt.
	@sudo echo "starting pyocd-gdbserver, logging to build/robot2015-gdb.log"
# now we can refresh the sudo timeout and start up the gdb server
	sudo -v && { sudo pyocd-gdbserver --allow-remote --port $(GDB_PORT) --reset-break \
	--target lpc1768 -S -G > build/robot2015-gdb.log 2>&1 & sudo echo $$! > $@; }

GDB_NO_CONN ?= 0
robot2015-gdb: robot2015 build/robot2015-gdb.pid
# use pyocd-gdbserver, and explicitly pass it the type of target we want to connect with,
# making sure that we enable semihosting and use gdb syscalls for the file io
	@trap 'sudo pkill -9 -P `cat build/robot2015-gdb.pid`; exit' TERM INT EXIT && \
	if [ $(GDB_NO_CONN) -eq 0 ]; then \
		arm-none-eabi-gdb build/firmware/firmware/robot2015/src-ctrl/robot2015_elf \
		  -ex "target remote localhost:$(GDB_PORT)" \
		  -ex "load" \
		  -ex "tbreak main" \
		  -ex "continue"; \
	else \
		while true; do sleep 10; done; \
	fi
