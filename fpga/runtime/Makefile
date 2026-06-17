# FB1 host runtime for niigo on AWS F2.
# Needs the aws-fpga SDK on the instance: source sdk_setup.sh first so
# SDK_DIR points at aws-fpga/sdk and libfpga_mgmt is built/installed.
#
#   source $AWS_FPGA_REPO_DIR/sdk_setup.sh
#   make -C fpga/runtime
#   sudo ./niigo_host --kernel kernel.bin --fs fs.img --debug

SDK_DIR ?= $(AWS_FPGA_REPO_DIR)/sdk
CC      ?= gcc
CFLAGS  := -std=gnu11 -O2 -Wall -Wextra -I$(SDK_DIR)/userspace/include
LDLIBS  := -lfpga_mgmt -lrt -lpthread

niigo_host: niigo_host.c niigo_ocl.h
	$(CC) $(CFLAGS) -o $@ niigo_host.c $(LDLIBS)

clean:
	rm -f niigo_host

.PHONY: clean
