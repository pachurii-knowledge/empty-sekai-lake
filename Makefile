# Local Phase 1 build flow.

SHELL = /bin/bash -o pipefail

OUTPUT_BASE_DIR ?= output
OUTPUT ?= $(OUTPUT_BASE_DIR)/simulation
SRC_DIR := $(shell readlink -m src)
SRC := $(shell find -L $(SRC_DIR) -type f \( -name '*.v' -o -name '*.sv' -o -name '*.vh' \) | sort)
SRC_SUBDIRS := $(shell find -L $(SRC_DIR) -type d | sort)

LAB_18447 ?= 4b
VALID_LABS := 1b 2 3 4a 4b

SIM_REGDUMP := $(OUTPUT)/simulation.reg
REF_REGDUMP := $(basename $(TEST)).reg
VERIFY_SCRIPT ?= sdiff
VERIFY_OPTIONS ?= --ignore-all-space --ignore-blank-lines
REGDUMP_COMPARE ?= python3 compare_regdump.py

RISCV_ENTRY_POINT ?= main
RISCV_CC ?= riscv64-unknown-elf-gcc
RISCV_OBJCOPY ?= riscv64-unknown-elf-objcopy
RISCV_OBJDUMP ?= riscv64-unknown-elf-objdump
# Target ISA for assembling tests. RV64=1 builds rv64 test binaries to match an
# RV64 simulator (make verilator-build RV64=1); default is rv32.
RISCV_ARCH ?= rv32i
RISCV_ABI  ?= ilp32
ifeq ($(RV64),1)
override RISCV_ARCH := rv64g
override RISCV_ABI  := lp64
endif
RISCV_CFLAGS ?= -static -nostdlib -nostartfiles -march=$(RISCV_ARCH) -mabi=$(RISCV_ABI) \
		-Wall -Wextra -std=c11 -pedantic -g \
		-Werror=implicit-function-declaration
RISCV_AS_LDFLAGS ?= -Wl,-e$(RISCV_ENTRY_POINT)
RISCV_LDFLAGS ?= -Wl,-T$(SRC_DIR)/test_program.ld -lgcc
RISCV_OBJCOPY_FLAGS ?= -O binary
RISCV_OBJDUMP_FLAGS ?= -d -M numeric,no-aliases $(addprefix -j ,.text \
		.ktext .data .bss .kdata .kbss)

TEST_STEM := $(notdir $(basename $(TEST)))
TEST_PARENT_DIR := $(notdir $(patsubst %/,%,$(dir $(TEST))))
TEST_ELF := $(OUTPUT)/$(TEST_STEM).elf
TEST_DISASSEMBLY := $(OUTPUT)/$(TEST_STEM).disassembly.s
TEST_OUTPUT_BIN := $(addprefix $(OUTPUT)/mem.,text.bin data.bin ktext.bin \
		kdata.bin)
TEST_OUTPUT_HEX := $(addprefix $(OUTPUT)/mem.,text.hex data.hex ktext.hex \
		kdata.hex)

ifeq ($(TEST_PARENT_DIR),benchmarks)
	RISCV_CFLAGS += -O -fno-inline
else ifeq ($(TEST_PARENT_DIR),perf_benchmarks)
	RISCV_CFLAGS += -O2 -fno-inline
else ifeq ($(TEST_PARENT_DIR),benchmarksO3)
	RISCV_CFLAGS += -O3 -fno-inline
else ifeq ($(TEST_PARENT_DIR),private)
	RISCV_CFLAGS += -O -fno-inline
else ifeq ($(TEST_PARENT_DIR),privateO3)
	RISCV_CFLAGS += -O3 -fno-inline
endif

n :=
r :=
g :=
b :=
u :=

.PHONY: all clean assemble assemble-clean check-lab-number-valid \
		check-test-defined check-riscv-toolchain ccd-m1-test ccd-wheel-test \
		$(TEST_OUTPUT_BIN) $(TEST_OUTPUT_HEX)

all: verilator-build

# ---- M1 two-core MOESI CCD coherence test (standalone; independent of the single-core build) ----
# Builds + runs tb_niigo_ccd_m1 (2 niigo_l1d_moesi agents + niigo_dir via niigo_ccd_top + an NMI
# memory) — a cross-core coherence program (expect "ALL CHECKS PASSED"). Verilator's -I doubles as
# the module search path (-y), so it auto-finds the CCD modules under src/mem.
ccd-m1-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-m1 --top-module tb_niigo_ccd_m1 \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_ccd_m1.sv
	$(OUTPUT_BASE_DIR)/ccd-m1/Vtb_niigo_ccd_m1

# ---- M3a wheel NoC fabric flit-level test (standalone; cmi_wheel = 4 core routers + hub) ----
ccd-wheel-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-wheel --top-module tb_cmi_wheel \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_cmi_wheel.sv
	$(OUTPUT_BASE_DIR)/ccd-wheel/Vtb_cmi_wheel

# ---- M3b two-core MOESI coherence over the wheel NoC (agents + dir + SerDes + routers) ----
ccd-wheel-coh-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-wheel-coh --top-module tb_niigo_ccd_wheel \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_ccd_wheel.sv
	$(OUTPUT_BASE_DIR)/ccd-wheel-coh/Vtb_niigo_ccd_wheel

$(OUTPUT):
	@mkdir -p $@

check-lab-number-valid:
ifeq ($(filter $(LAB_18447),$(VALID_LABS)),)
	$(error Invalid LAB_18447='$(LAB_18447)'; expected one of $(VALID_LABS))
endif

check-test-defined:
ifeq ($(strip $(TEST)),)
	$(error TEST was not specified)
endif

clean: verilator-clean
	@rm -rf $(OUTPUT_BASE_DIR)

check-riscv-toolchain:
ifeq ($(shell which $(RISCV_CC) 2> /dev/null),)
	$(error $(RISCV_CC) was not found in PATH)
endif
ifeq ($(shell which $(RISCV_OBJCOPY) 2> /dev/null),)
	$(error $(RISCV_OBJCOPY) was not found in PATH)
endif
ifeq ($(shell which $(RISCV_OBJDUMP) 2> /dev/null),)
	$(error $(RISCV_OBJDUMP) was not found in PATH)
endif

assemble: $(TEST_OUTPUT_BIN) $(TEST_OUTPUT_HEX) $(TEST_DISASSEMBLY)

$(TEST_ELF): $(TEST) $(SRC_DIR)/test_program.ld | $(OUTPUT) \
		check-test-defined check-riscv-toolchain
	@printf "Assembling test $u$(TEST)$n into $(OUTPUT)...\n"
	@if [[ "$(TEST)" == *.c ]]; then \
		$(RISCV_CC) $(RISCV_CFLAGS) $(SRC_DIR)/crt0.S $(TEST) \
			$(RISCV_LDFLAGS) -o $@ |& tee $(OUTPUT)/assemble.log; \
	else \
		$(RISCV_CC) $(RISCV_CFLAGS) $(TEST) $(RISCV_LDFLAGS) \
			$(RISCV_AS_LDFLAGS) -o $@ |& tee $(OUTPUT)/assemble.log; \
	fi

$(OUTPUT)/mem.text.bin: $(TEST_ELF)
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .text $< $@ |& \
		tee -a $(OUTPUT)/assemble.log

$(OUTPUT)/mem.data.bin: $(TEST_ELF)
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .data -j .bss \
		--set-section-flags .bss=alloc,load,contents $< $@

$(OUTPUT)/mem.ktext.bin: $(TEST_ELF)
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .ktext $< $@ |& \
		tee -a $(OUTPUT)/assemble.log

$(OUTPUT)/mem.kdata.bin: $(TEST_ELF)
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .kdata -j .kbss \
		--set-section-flags .kbss=alloc,load,contents $< $@

$(OUTPUT)/mem.%.hex: $(OUTPUT)/mem.%.bin
	@if [[ -s "$<" ]]; then od -An -tx4 -v "$<" > "$@"; else : > "$@"; fi

$(TEST_DISASSEMBLY): $(TEST_ELF)
	@$(RISCV_OBJDUMP) $(RISCV_OBJDUMP_FLAGS) $< > $@ |& \
		tee -a $(OUTPUT)/assemble.log
	@printf "Disassembly written to $u$@$n.\n"

assemble-clean:
	@rm -f $(OUTPUT)/mem.*.bin $(OUTPUT)/mem.*.hex $(OUTPUT)/*.elf \
		$(OUTPUT)/*.disassembly.s \
		$(OUTPUT)/assemble.log

include src/verilator.mk
