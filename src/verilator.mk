# Verilator simulation flow for local Phase 1 development.
#
# Usage:
#   make verilator-build
#   make verilator-sim
#   make verilator-verify TEST=path/to/reference-test.S
#
# Place mem.text.bin, mem.data.bin, mem.ktext.bin, and mem.kdata.bin in
# $(OUTPUT) before running verilator-sim.

VERILATOR ?= verilator

VERILATOR_TOP_MODULE := top
VERILATOR_OBJ_DIR := verilator_obj
VERILATOR_OBJ_PATH := $(OUTPUT)/$(VERILATOR_OBJ_DIR)
VERILATOR_COMPILE_LOG := verilator_compilation.log
VERILATOR_SIM_LOG := verilator_simulation.log
VERILATOR_EXECUTABLE := $(VERILATOR_OBJ_PATH)/V$(VERILATOR_TOP_MODULE)

VERILATOR_SYNTHESIS_ONLY_SRC := $(SRC_DIR)/riscv_core_timing.sv \
		$(SRC_DIR)/sram_synthesis.sv
CVFPU_DIR := $(SRC_DIR)/cvfpu
VERILATOR_SRC := $(filter-out $(VERILATOR_SYNTHESIS_ONLY_SRC) $(CVFPU_DIR)/%, \
		$(filter %.v %.sv,$(SRC)))
CVFPU_SRC := \
		$(CVFPU_DIR)/src/fpnew_pkg.sv \
		$(CVFPU_DIR)/src/fpnew_cast_multi.sv \
		$(CVFPU_DIR)/src/fpnew_classifier.sv \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/clk/rtl/gated_clk_cell.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_ctrl.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_ff1.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_pack_single.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_prepare.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_round_single.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_special.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_srt_single.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fdsu/rtl/pa_fdsu_top.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fpu/rtl/pa_fpu_dp.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fpu/rtl/pa_fpu_frbus.v \
		$(CVFPU_DIR)/vendor/opene906/E906_RTL_FACTORY/gen_rtl/fpu/rtl/pa_fpu_src_type.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_ctrl.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_double.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_ff1.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_pack.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_prepare.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_round.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_scalar_dp.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_srt_radix16_bound_table.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_srt_radix16_with_sqrt.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_srt.v \
		$(CVFPU_DIR)/vendor/openc910/C910_RTL_FACTORY/gen_rtl/vfdsu/rtl/ct_vfdsu_top.v \
		$(CVFPU_DIR)/src/fpnew_divsqrt_th_32.sv \
		$(CVFPU_DIR)/src/fpnew_divsqrt_th_64_multi.sv \
		$(CVFPU_DIR)/src/fpnew_divsqrt_multi.sv \
		$(CVFPU_DIR)/src/fpnew_fma.sv \
		$(CVFPU_DIR)/src/fpnew_fma_multi.sv \
		$(CVFPU_DIR)/src/fpnew_noncomp.sv \
		$(CVFPU_DIR)/src/fpnew_opgroup_block.sv \
		$(CVFPU_DIR)/src/fpnew_opgroup_fmt_slice.sv \
		$(CVFPU_DIR)/src/fpnew_opgroup_multifmt_slice.sv \
		$(CVFPU_DIR)/src/fpnew_rounding.sv \
		$(CVFPU_DIR)/src/fpnew_top.sv
VERILATOR_DESIGN_SRC := $(CVFPU_SRC) $(sort $(VERILATOR_SRC))

VERILATOR_INC_FLAGS := $(addprefix -I,$(SRC_SUBDIRS))
VERILATOR_CFLAGS ?= --sv --timing --binary -Wno-fatal \
		--top-module $(VERILATOR_TOP_MODULE) --Mdir $(VERILATOR_OBJ_DIR) \
		--output-split 5000 --output-split-cfuncs 5000 \
		-DSIMULATION_18447 \
		-DLAB_18447='"4b"'

ifeq ($(SUPERSCALAR),4)
	VERILATOR_CFLAGS += -DSUPERSCALAR_4WIDE
endif

ifeq ($(OOO),1)
	VERILATOR_CFLAGS += -DOOO_4WIDE
endif

# L1=1 enables the L1I cache (phase C1). L1D=1 additionally enables the
# write-back L1D + PTW-through-L1D (phase C2) and implies L1_CACHES. AXI=1 routes
# the NMI bus through the AXI4-512 bridge + sim slave (requires the caches).
# All OoO-build only.
ifeq ($(L1),1)
	VERILATOR_CFLAGS += -DL1_CACHES
endif

ifeq ($(L1D),1)
	VERILATOR_CFLAGS += -DL1_CACHES -DL1D_CACHE
endif

# CCD=1 (M3d) puts the grant-and-go MOESI L1D agent (niigo_l1d_gg via
# niigo_ccd_gg_direct #(.NACTIVE(1)) + niigo_dir_gg) on the OoO core's D-side in
# place of the C2 L1D. Reuses the L1I + shared-NMI backend (-DL1_CACHES). Single
# core; coherence inert. Mutually exclusive with L1D=1 (do NOT also set L1D).
ifeq ($(CCD),1)
	VERILATOR_CFLAGS += -DCCD_AGENT -DL1_CACHES
endif

# L2=1 (plans/l2-integration.md) interposes the transparent write-back NINE L2
# (niigo_l2) on the directory's NMI memory leg via niigo_ccd_gg_direct's L2_ENABLE
# param (-DL2_CACHE). Compose with CCD=1 (the single-core CCD arm) or an SMP ccd-*
# target. Value-transparent: changes only memory-leg latency, not results.
ifeq ($(L2),1)
	VERILATOR_CFLAGS += -DL2_CACHE
endif

ifeq ($(AXI),1)
	VERILATOR_CFLAGS += -DAXI_MEMSYS
endif

ifeq ($(AGENT_DEBUG),1)
	VERILATOR_CFLAGS += -DAGENT_DEBUG
endif

# RV64=1 selects the 64-bit datapath (XLEN=64, Sv39). Composes with OOO/
# SUPERSCALAR; default (unset) builds RV32G. See plans/rv64-linux.md (Track 2).
ifeq ($(RV64),1)
	VERILATOR_CFLAGS += -DRV64
endif

# RVC=1 enables the RV64C compressed (16-bit) instruction extension: a two-wide
# expand-before-decode realign frontend (rvc_realign) + a pure-combinational
# 16->32 expander (rvc_expand), all -DRVC gated. RV64GC only -- RVC requires
# RV64 (the RV32C-only encoding slots are not implemented). Composes with
# OOO/L1/L1D/AXI/CCD (purely a core-frontend change). The non-RVC build stays
# bit-identical. See the RV64C plan.
ifeq ($(RVC),1)
ifneq ($(RV64),1)
$(error RVC=1 requires RV64=1 (RV64GC only); the RV32C encoding slots are not implemented)
endif
	VERILATOR_CFLAGS += -DRVC
endif

# REALIGN4=1 widens the RV64C expand-before-decode realigner from 2 to 4 lanes
# (plans/ooo-perf.md P4), so the compressed frontend can feed all 4 backend
# dispatch slots. It reconciles with the P2b offset-precise BTB termination (the
# terminate/parcel accounting gain lane-2/3 terms). Requires RVC; default OFF is
# behaviourally identical to the 2-wide realigner (all gated by -DREALIGN4). OOO only.
ifeq ($(REALIGN4),1)
ifneq ($(RVC),1)
$(error REALIGN4=1 requires RVC=1 (it widens the RV64C realigner))
endif
	VERILATOR_CFLAGS += -DREALIGN4
endif

# DEEP_WINDOW=1 grows PHYS_REGS 64->128 (plans/ooo-perf.md P6), giving the physical-register
# free list burst headroom over the 32-entry ROB. It removes the P4 4-wide qsort freelist
# starvation (freelist_stall 37% -> ~0) where the 2-stage commit frees regs slower than
# 4-wide dispatch allocates. 128 (not 96) because free_list.sv is a power-of-2 ring buffer.
# phys_reg_t auto-widens 6->7; the ROB is left at 32. Default OFF is bit-identical. OOO only.
ifeq ($(DEEP_WINDOW),1)
	VERILATOR_CFLAGS += -DDEEP_WINDOW
endif

# ASIC=1 selects the ASAP7 7nm ASIC target (-DNIIGO_ASIC), mirroring RV64=1 for
# datapath width. It flips the target-divergent, RESULTS-IDENTICAL knobs the ASIC
# flow needs -- CVFPU FMA 2->3 pipe stages (niigo_fp_unit) and RAS depth 128->32
# (riscv_core_ooo) -- so a functional Verilator run can exercise the ASIC config.
# The default (ASIC unset) is the FPGA / functional-sim target (FMA=2, RAS=128).
# The physical SRAM-macro mapping is a SEPARATE opt-in (-DNIIGO_SRAM_MACRO, which
# needs the ASAP7 cell models) and is left to the OpenROAD flow, not this build.
ifeq ($(ASIC),1)
	VERILATOR_CFLAGS += -DNIIGO_ASIC
endif

# BTB=1 enables the fetch-directed branch target buffer (plans/ooo-perf.md P2a):
# src/btb.sv, looked up on pc_next so a hit steers the next fetch to the target
# (N->T, no wrong-path block), verified + trained at decode, flush suppressed for
# block-ending agrees. Prediction-only; default OFF is bit-identical (all gated by
# -DBTB). OOO only.
ifeq ($(BTB),1)
	VERILATOR_CFLAGS += -DBTB
endif

# XLATE_BYPASS=1 lets a DTLB-hit head memory op issue the SAME cycle it presents,
# bypassing the FB2b registered-translate stage (plans/ooo-perf.md P3 lever 1). Cuts
# the per-mem-op xlate-wait bubble; re-opens the LSQ-head->DTLB->DataPMP->issue placed
# path (Fmax cost). Default OFF is bit-identical (all gated by -DXLATE_BYPASS). OOO only.
ifeq ($(XLATE_BYPASS),1)
	VERILATOR_CFLAGS += -DXLATE_BYPASS
endif

.PHONY: verilator-build verilator-sim verilator-verify verilator-clean \
		verilator-check-compiler

verilator-build: $(VERILATOR_EXECUTABLE)

$(VERILATOR_EXECUTABLE): $(VERILATOR_DESIGN_SRC) | $(OUTPUT) \
		verilator-check-compiler check-lab-number-valid
	@printf "Compiling design with Verilator into $u$(VERILATOR_OBJ_DIR)$n...\n"
	@cd $(OUTPUT) && $(VERILATOR) $(VERILATOR_CFLAGS) $(VERILATOR_INC_FLAGS) \
		$(VERILATOR_DESIGN_SRC) |& tee $(VERILATOR_COMPILE_LOG)
	@printf "\nVerilator compilation has completed. The compilation log can be "
	@printf "found at $u$(OUTPUT)/$(VERILATOR_COMPILE_LOG)$n\n"
	@printf "The Verilator executable can be found at $u$@$n.\n"

VERILATOR_MEMORY_IMAGES := $(addprefix $(OUTPUT)/,mem.text.bin mem.data.bin \
		mem.ktext.bin mem.kdata.bin)

verilator-sim: assemble $(VERILATOR_EXECUTABLE) | $(OUTPUT)
	@printf "Simulating test $u$(TEST)$n with Verilator in $u$(OUTPUT)$n...\n"
	@cd $(OUTPUT) && ./verilator_obj/V$(VERILATOR_TOP_MODULE) |& tee $(VERILATOR_SIM_LOG)
	@printf "\nVerilator simulation has completed. The simulation log can be "
	@printf "found at $u$(OUTPUT)/$(VERILATOR_SIM_LOG)$n\n"
	@printf "The simulator register dump can be found at $u$(SIM_REGDUMP)$n\n"

verilator-verify: verilator-sim $(REF_REGDUMP) check-test-defined
	@printf "\n"
	@if { \
		if [[ "$(TEST)" == *.c ]]; then \
			$(REGDUMP_COMPARE) $(SIM_REGDUMP) $(REF_REGDUMP) c; \
		else \
			$(VERIFY_SCRIPT) $(VERIFY_OPTIONS) $(SIM_REGDUMP) $(REF_REGDUMP); \
		fi; \
	} &> /dev/null; then \
		printf "$gCorrect! The Verilator register dump matches the reference.$n\n"; \
	else \
		printf "\n%-67s\t%s\n\n" "$u$(SIM_REGDUMP)$n" "$u$(REF_REGDUMP)$n"; \
		if [[ "$(TEST)" == *.c ]]; then \
			$(REGDUMP_COMPARE) $(SIM_REGDUMP) $(REF_REGDUMP) c; \
		else \
			$(VERIFY_SCRIPT) $(VERIFY_OPTIONS) $(SIM_REGDUMP) $(REF_REGDUMP); \
		fi; \
		printf "$rIncorrect! The Verilator register dump does not match the "; \
		printf "reference.$n\n"; \
		exit 1; \
	fi

verilator-clean:
	@printf "Cleaning up Verilator files...\n"
	@rm -rf $(VERILATOR_OBJ_PATH) $(OUTPUT)/$(VERILATOR_COMPILE_LOG) \
		$(OUTPUT)/$(VERILATOR_SIM_LOG)

verilator-check-compiler:
ifeq ($(shell which $(VERILATOR) 2> /dev/null),)
	@printf "$rError: $u$(VERILATOR)$n$r was not found in your $bPATH$n$r.\n$n"
	@printf "Install Verilator 5.x, then run $bmake verilator-build$n.\n"
	@exit 1
endif
