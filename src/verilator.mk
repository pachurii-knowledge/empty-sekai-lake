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

# PERF=1 = the canonical OoO performance build (plans/ooo-perf.md). One umbrella flag for the
# full landed-lever stack on the REALISTIC memory config (L1D). Passthrough is NOT used for perf:
# its fixed 8-cycle-every-load fakes a memory-bound signature (LOADWAIT collapses to the identity
# loads x (latency-1)), which flips window/memory/translate conclusions vs L1D -- e.g. XLATE_BYPASS
# reads modest on passthrough but +13% mean on L1D. See the P3 mem-lever recon. Expands to RV64GC +
# L1D + the gated perf levers; each flag still composes individually for A/B isolation. NB: this is a
# FUNCTIONAL sim perf build -- XLATE_BYPASS and LSQ_MLP2 carry an Fmax cost, so an FPGA/ASIC build
# should drop them. Must precede the sub-flag ifeq blocks so they see these vars.
ifeq ($(PERF),1)
	RV64 := 1
	OOO := 1
	RVC := 1
	L1D := 1
	REALIGN4 := 1
	DEEP_WINDOW := 1
	BIG_IQ := 1
	BIG_ROB := 1
	BIG_LSQ := 1
	BTB := 1
	XLATE_BYPASS := 1
	FP_OOO := 1
	ALU4 := 1
	LSQ_MLP2 := 1
endif

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

# P6b window-depth structure resizes (plans/ooo-perf.md P6). Each grows one OoO
# window structure; defaults are bit-identical. INT_IQ (BIG_IQ) is a collapsing slot
# queue (16->24, any size legal). ACTIVE_LIST/ROB (BIG_ROB) and MEM_Q/LSQ (BIG_LSQ)
# are power-of-2 ring buffers so they take only pow2 sizes (ROB 32->64, LSQ 16->32).
# BIG_ROB=64 needs PHYS_REGS >= 32+64 = 96 (free-list floor) => it REQUIRES DEEP_WINDOW
# (PHYS_REGS=128). OOO only. The three grow the same entries_q arrays whose whole-array
# NBAs are now element-wise loops (Verilator V3Delayed workaround), default unchanged.
ifeq ($(BIG_IQ),1)
	VERILATOR_CFLAGS += -DBIG_IQ
endif
ifeq ($(BIG_LSQ),1)
	VERILATOR_CFLAGS += -DBIG_LSQ
endif
ifeq ($(BIG_ROB),1)
ifneq ($(DEEP_WINDOW),1)
$(error BIG_ROB=1 requires DEEP_WINDOW=1: a 64-entry ROB needs PHYS_REGS>=96 (free-list deadlock floor); DEEP_WINDOW sets PHYS_REGS=128)
endif
	VERILATOR_CFLAGS += -DBIG_ROB
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

# CF_OOO=1 (-DCF_OOO): out-of-order control-flow issue. The baseline (int_issue_queue.sv)
# lets a CF op issue only once every older branch has resolved, so only the OLDEST
# unresolved branch may execute -- branch resolve, and hence branch-checkpoint
# reclamation, is fully serialized. That gate was an admitted stop-gap ("conservatively
# handle nested branches for now", 7264445), not a correctness requirement; branch_stack
# is an order-agnostic pool whose frees are keyed by resolve_id. CF_OOO demotes it from a
# BAN to a PRIORITY: a younger branch may take the (<=1/cycle) CF issue slot only when the
# oldest unresolved branch is not ready to use it, so the oldest branch is never delayed.
# Pairs with BIG_BSTACK (a faster-draining pool wants more slots to run ahead into).
# Default OFF is bit-identical (the else-arm is the verbatim baseline expression). OOO only.
# NOTE: requires the active_list restore_count() fix (full/empty ring alias) -- CF_OOO makes
# that latent ROB-wipe reachable. Costs: bounded wrong-path predictor training (a branch may
# now execute in a mispredict shadow, which the baseline made impossible).
ifeq ($(CF_OOO),1)
	VERILATOR_CFLAGS += -DCF_OOO
endif

# JAL_NO_CKPT=1 (-DJAL_NO_CKPT): stop allocating a branch checkpoint for JAL
# (PC_uncond: JAL/c.j/c.jal). A JAL is always correctly predicted (predicted_pc ==
# pc+imm == target), so its checkpoint is provably never restored -- dead weight that
# also inserts a branch_mask bit serializing younger branches and, on a full stack,
# spuriously stalls dispatch. Drops the JAL's checkpoint allocate + branch resolve and
# lets a JAL dispatch into a full branch stack (a gated lane_needs_ckpt port on
# ooo_dispatch_control). lane_is_branch stays TRUE for a JAL, so dispatch-group
# termination and the B2 predictor break are unchanged; RAS/GHR speculative state is
# captured by the next younger branch's checkpoint, not the JAL's. Default OFF is
# bit-identical. OOO only; pairs with CF_OOO/BIG_BSTACK (frees the pool they contend for).
ifeq ($(JAL_NO_CKPT),1)
	VERILATOR_CFLAGS += -DJAL_NO_CKPT
endif

# DUAL_BRANCH_COUNT=1 (-DDUAL_BRANCH_COUNT): measurement-only perf counters that
# upper-bound the dispatch throughput a 2nd-branch-per-group would recover
# (grp_branch_cut / grp_2nd_branch in perf.txt). No datapath effect; used to make the
# DUAL_BRANCH go/no-go empirical before writing any datapath RTL. OOO only.
ifeq ($(DUAL_BRANCH_COUNT),1)
	VERILATOR_CFLAGS += -DDUAL_BRANCH_COUNT
endif

# DFE_STATS=1 (-DDFE_STATS): decoupled-frontend (FTQ) S0 instrumentation. Measures how
# much of the sync-read predict_stall bubble is actually CONVERTIBLE (predict_stall_sole
# = the only dispatch inhibitor + backend not full) -- the go/no-go evidence for building
# the FTQ frontend (plans/decoupled-fetch-ftq.md). No datapath effect. OOO only.
ifeq ($(DFE_STATS),1)
	VERILATOR_CFLAGS += -DDFE_STATS
endif

# DFE_S1=1 (-DDFE_S1): decoupled-frontend (FTQ) Stage 1a -- an INERT fetch-side branch
# predecode + one-directional equivalence assertion (predecoded branchPC == the dispatch-
# directed TAGE/ITTAGE lookup PC). Proves fetch-side branch identification is byte-identical
# to today's dispatch-directed key -- the load-bearing accuracy premise for the FTQ (S2).
# No consumer, no new flops; OFF is source-unchanged bit-identical. A fired assertion kills
# the FTQ approach cheaply. See plans/decoupled-fetch-ftq.md. OOO only.
ifeq ($(DFE_S1),1)
	VERILATOR_CFLAGS += -DDFE_S1
endif

# BIG_BSTACK=1 (-DBIG_BSTACK): branch checkpoints 4 -> 8 (plans/ooo-perf.md P7).
# branch_mask_t (4->8b) and branch_id_t auto-widen through every mask-holding
# structure; default OFF (the verbatim 4) is bit-identical. Re-applied from the P7
# branch commit d3180c8, whose verdict was NEUTRAL on xv6/fibrec/branchy/qsort --
# the bstack stall was eliminated but yielded no IPC (a symptom, not the binder).
# Re-tested here because Dhrystone is far more branch-dense (bstack_full = 37% of
# cycles vs xv6's 14.5%), which is a regime P7 never measured. Costs a doubled
# abort_mask broadcast fanout + per-checkpoint rename-map copy (FB2b routed-WNS net).
ifeq ($(BIG_BSTACK),1)
	VERILATOR_CFLAGS += -DBIG_BSTACK
endif

# FP_OOO=1 de-serializes floating-point ops (plans/ooo-perf.md P5b): drops the
# machine-wide dispatch quiesce every FP op raises today and replaces it with a
# single-producer arch-FPR scoreboard (one in-flight writer per FPR via a WAW
# dispatch-stall; producer branch_mask aged by ~reset_mask for abort recovery) +
# an fflags-read drain interlock. Default OFF is bit-identical (all gated by
# -DFP_OOO). OOO only.
ifeq ($(FP_OOO),1)
	VERILATOR_CFLAGS += -DFP_OOO
endif

# ALU4=1 adds a 3rd integer ALU issue port (ALU_ISSUE_PORTS 2->3, plans/ooo-perf.md)
# so up to 3 independent ALU ops issue/cycle -- relieves the 2-ALU-port throughput
# binder on integer-ILP code (alu_ilp +25.7%; neutral where ALU1 is already idle).
# Widens the IQ ALU pick (parameterized N-pick, bit-identical at 2 ports), adds a
# 3rd ALU pipe (CSR confined to ALU0/1) + regfile read ports + a 3rd writeback
# source. Folded into PERF. Default OFF is bit-identical. OOO only; area/Fmax cost
# (like XLATE_BYPASS) -> an FPGA/ASIC build should drop it.
ifeq ($(ALU4),1)
	VERILATOR_CFLAGS += -DALU4
endif

# LSQ_MLP2=1 (Track A, plans/track-a-mlp.md): non-blocking L1D / memory-level
# parallelism -- lifts the LSQ single-outstanding-load limit to 2 (LSQ_MLP=2).
# OoO only; the win needs a real cache (L1D), but it compiles bit-identically on
# passthrough/L1 too (the id round-trip threads the passthrough d_q FIFO as well).
# INCOMPATIBLE with CCD: a coherent D-side would pin LSQ_MLP=1 anyway (2 in-flight
# loads open an RVWMO load-load reorder window with no 9.11 squash), and the CCD
# agent memsys arm has no id round-trip, so combining them is disallowed outright.
# Default OFF is bit-identical (all gated by -DLSQ_MLP2). Folded into PERF; functional sim
# lever like XLATE_BYPASS (an FPGA/ASIC build likely drops it -- Fmax). The registered-aim
# per-load issue bubble is fixed (combinational head_owns_port), so it is net-positive:
# survey ~0, independent-miss streams +~55%, aggregate +6.45% under fuzz-16 L1D.
ifeq ($(LSQ_MLP2),1)
ifeq ($(CCD),1)
$(error LSQ_MLP2=1 is incompatible with CCD=1: the coherent D-side pins LSQ_MLP=1 and the CCD memsys arm carries no dmem txn-id. Use L1D=1 (or PERF=1).)
endif
	VERILATOR_CFLAGS += -DLSQ_MLP2
endif

# LSQ_MLP_STAT=1: instrumented overlap counters (ip_fires/two_out/steps -> LSQ-MLP-STAT
# final $display) that PROVE MLP=2 engages. Implies LSQ_MLP2. Diagnostic-only; keep it
# OUT of PERF so the canonical build stays lean.
ifeq ($(LSQ_MLP_STAT),1)
	VERILATOR_CFLAGS += -DLSQ_MLP2 -DLSQ_MLP_STAT
endif

# COMMIT1=1 (-DCOMMIT_1STAGE): revert active_list's commit path from the FB2b 2-stage
# (registered recovery-root, sliding-window present) to the pre-FB2b 1-stage combinational
# commit (present-at-head, no register). The 2-stage delays every commit-gated dependency
# release by a cycle -- and the FP_OOO scoreboard clears fpr_busy at COMMIT, so FP-dense
# code pays it repeatedly (fpkernel +9.15%, aggregate +1.57% on fuzz-16 L1D). Default OFF
# is the 2-stage (bit-identical). Functional sim lever like XLATE_BYPASS/LSQ_MLP2 -- it
# re-opens the FB2b recovery-root cone (WNS +0.99ns), so an FPGA/ASIC build must keep it OFF.
ifeq ($(COMMIT1),1)
	VERILATOR_CFLAGS += -DCOMMIT_1STAGE
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
