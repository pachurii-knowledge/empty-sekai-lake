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

# ---- M3c grant-and-go protocol on the behavioural direct interconnect (protocol validation) ----
ccd-gg-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-gg --top-module tb_niigo_ccd_gg \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_ccd_gg.sv
	$(OUTPUT_BASE_DIR)/ccd-gg/Vtb_niigo_ccd_gg

# ---- 4-core grant-and-go directed protocol tests (NACTIVE=4): the SAME S1-S6/C1-C8 programs PLUS
#      G1 (multi-sharer ack-to-requester, acks>1), S4 (owner-in-O upgrade vs peer GetM), and S3
#      (dirty-shared O line evicted while a peer snoops) -- the paths unreachable at 2 cores. ----
ccd-gg4-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-gg4 --top-module tb_niigo_ccd_gg \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -DNC4 -Isrc -Isrc/mem tests/tb_niigo_ccd_gg.sv
	$(OUTPUT_BASE_DIR)/ccd-gg4/Vtb_niigo_ccd_gg

# ---- M3c-D grant-and-go MOESI coherence over the real wheel NoC (dir/agent on the hub-funnel + ring) ----
ccd-gg-wheel-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-gg-wheel --top-module tb_niigo_ccd_gg_wheel \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_ccd_gg_wheel.sv
	$(OUTPUT_BASE_DIR)/ccd-gg-wheel/Vtb_niigo_ccd_gg_wheel

# ---- M3b two-core MOESI coherence over the wheel NoC (agents + dir + SerDes + routers) ----
ccd-wheel-coh-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/ccd-wheel-coh --top-module tb_niigo_ccd_wheel \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_niigo_ccd_wheel.sv
	$(OUTPUT_BASE_DIR)/ccd-wheel-coh/Vtb_niigo_ccd_wheel

# ---- M3d Stage 3 reservation-coherence-kill validation: ONE real riscv_core_ooo (core 0) + a
#      behavioural peer (core 1) over niigo_ccd_gg_direct #(.NACTIVE(2)). Builds the FULL OoO core
#      (top overridden to tb_ccd_stage3) + serves the assembled litmus via a behavioural ifetch. ----
ccd-stage3-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-stage3 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_stage3.sv)
	@echo "--- contended (peer writes the reserved line -> sc.w must FAIL) ---"
	$(OUTPUT_BASE_DIR)/ccd-stage3/Vtop
	@echo "--- negative control (peer writes a different line -> sc.w must SUCCEED) ---"
	$(OUTPUT_BASE_DIR)/ccd-stage3/Vtop +nokill

# ---- M4 S4: real multi-core SMP -- N real riscv_core_ooo cores over niigo_ccd_gg_direct #(.NACTIVE(N))
#      running an LR/SC spinlock; the shared counter == N*ITERS iff coherence + reservation-kill hold. ----
ccd-smp-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp/Vtop

# ---- M4 #4: 4-core scale-up of the LR/SC spinlock (NCORE=4 -> counter == 4*ITERS) ----
ccd-smp4-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp4 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DNCORE4 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp4/Vtop

# ---- M4 #3 AMO atomicity: 2 real cores contend amoadd.w on a shared counter ----
ccd-smp-amo-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-amo \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_amo.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-amo/Vtop

# ---- M4 #4: 4-core AMO atomicity scale-up (NCORE=4 -> counter == 4*ITERS) ----
ccd-smp-amo4-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-amo4 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNCORE4 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_amo.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-amo4/Vtop

# ---- M4-S6b cross-hart IPI: 2 real cores + ONE shared CLINT (NIIGO_EXT_DEVICES) ----
ccd-smp-ipi-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-ipi \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_ipi.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-ipi/Vtop

# ---- M4 #4: 4-core IPI broadcast (hart 0 -> msip[1..3]; each receiver traps) ----
ccd-smp-ipi4-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-ipi4 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DNCORE4 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_ipi.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-ipi4/Vtop

# ---- post-M4 P1: RV64-on-CCD. The same three SMP litmi at XLEN=64 (-DRV64): proves the
#      grant-and-go fabric + agents + the B9 LR/SC and agent-authoritative AMO fixes hold at
#      64-bit (lr.d/sc.d/ld/sd, amoadd.d). Prerequisite for the RV64 xv6-SMP boot path. ----
ccd-smp-rv64-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-rv64 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-rv64/Vtop

ccd-smp-amo-rv64-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-amo-rv64 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_amo.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-amo-rv64/Vtop

ccd-smp-ipi-rv64-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smp-ipi-rv64 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smp_ipi.sv)
	$(OUTPUT_BASE_DIR)/ccd-smp-ipi-rv64/Vtop

# ---- post-M4 P4: cross-core self-modifying-code litmus. NCORE real cores, each with a REAL
#      l1_icache behind the directory; the remote-dirty I-fetch (probe local L1D, COP_LOAD on
#      miss, probe-serve) lets hart 1 fetch hart 0's freshly-patched (dirty) code line coherently
#      with no fence.i -- the mechanism xv6-SMP needs for cross-hart exec. RESULT==0x222. ----
ccd-smc-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smc \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smc.sv)
	$(OUTPUT_BASE_DIR)/ccd-smc/Vtop

ccd-smc-rv64-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smc-rv64 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smc.sv)
	$(OUTPUT_BASE_DIR)/ccd-smc-rv64/Vtop

ccd-smc4-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-smc4 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNCORE4 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_smc.sv)
	$(OUTPUT_BASE_DIR)/ccd-smc4/Vtop

# ---- post-M4 P4-incr2: the reusable niigo_ccd_memsys multi-core memsys, validated by the SMC
#      litmus run THROUGH the module (per-core L1I+launch-adapter+iref+probe factored in; PTW +
#      device-bypass paths included for the xv6-SMP harness). RESULT==0x222. ----
ccd-memsys-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-memsys \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_memsys.sv)
	$(OUTPUT_BASE_DIR)/ccd-memsys/Vtop

ccd-memsys-rv64-test:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-memsys-rv64 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_memsys.sv)
	$(OUTPUT_BASE_DIR)/ccd-memsys-rv64/Vtop

# ---- post-M4 P6: xv6-SMP boot harness (build only; run from a staged xv6 image dir).
#      NCORE real RV64 cores over niigo_ccd_memsys + nmi_mem_adapter->main_memory + a shared
#      CLINT/PLIC/UART hub. Stage the image (scripts/load_elf_mem.py + fs.img manifest) into a
#      run dir, then:  cd output/xv6m2 && <Mdir>/Vtop +no_ecall_halt +uart_in=$'ls\n' ----
ccd-xv6-build:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-xv6 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DRV64 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_xv6.sv)

ccd-xv6-1-build:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-xv6-1 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DRV64 -DNCORE1 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_xv6.sv)

# NCORE=4 xv6-SMP harness (for usertests-under-4-cores bring-up). Large build: the 2-OoO-core
# Vtop already needs ~10GB per big file -- 4 cores is bigger, so build with a bounded, detached
# Mdir make, e.g.:  make -C output/ccd-xv6-4 -j2 -f Vtop.mk  (see OVERNIGHT_BUGLOG.md / plans/smp-4core-bug-surface.md).
ccd-xv6-4-build:
	$(VERILATOR) --sv --timing --binary -j 0 -Wno-fatal --top-module top \
		--Mdir $(OUTPUT_BASE_DIR)/ccd-xv6-4 \
		-DSIMULATION_18447 -DLAB_18447='"4b"' -DOOO_4WIDE -DCCD_AGENT -DL1_CACHES -DNIIGO_EXT_DEVICES -DRV64 -DNCORE4 \
		$(VERILATOR_INC_FLAGS) \
		$(filter-out %/testbench.sv,$(VERILATOR_DESIGN_SRC)) $(abspath tests/tb_ccd_xv6.sv)

# ---- M4-S6a multi-hart shared CLINT/PLIC directed test (standalone; clint NUM_HARTS=4 + plic NCTX=8) ----
clint-plic-smp-test:
	verilator --binary -j 0 --Mdir $(OUTPUT_BASE_DIR)/clint-plic-smp --top-module top \
		-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE \
		-DLAB_18447='"4b"' -Isrc -Isrc/mem tests/tb_clint_plic_smp.sv
	$(OUTPUT_BASE_DIR)/clint-plic-smp/Vtop

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
