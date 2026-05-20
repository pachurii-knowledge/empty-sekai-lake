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
VERILATOR_SRC := $(filter-out $(VERILATOR_SYNTHESIS_ONLY_SRC), \
		$(filter %.v %.sv,$(SRC)))
VERILATOR_DESIGN_SRC := $(sort $(VERILATOR_SRC))

VERILATOR_INC_FLAGS := $(addprefix -I,$(SRC_SUBDIRS))
VERILATOR_CFLAGS ?= --sv --timing --binary -Wno-fatal \
		--top-module $(VERILATOR_TOP_MODULE) --Mdir $(VERILATOR_OBJ_DIR) \
		-DSIMULATION_18447 \
		-DLAB_18447='"4b"'

ifeq ($(SUPERSCALAR),4)
	VERILATOR_CFLAGS += -DSUPERSCALAR_4WIDE
endif

ifeq ($(OOO),1)
	VERILATOR_CFLAGS += -DOOO_4WIDE
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
