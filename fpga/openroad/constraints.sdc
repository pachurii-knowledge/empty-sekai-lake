# constraints.sdc — niigo_soc boundary constraints (ASAP7 bring-up skeleton).
# Period is a placeholder; tighten once Tier-1 closes. Units: ns (ASAP7 Liberty is ns/fF).

set clk_period_ns [expr {[info exists ::env(CLK_PERIOD_NS)] ? $::env(CLK_PERIOD_NS) : 2.0}]
create_clock -name clk -period $clk_period_ns [get_ports clk]

# Async active-low reset: synchronize at the boundary (see SCOPING.md §3a) and false-path it.
set_false_path -from [get_ports rst_l]

# AXI4-512 master + control as primary IO: budget ~30% of the period at the boundary.
set io_delay [expr {0.30 * $clk_period_ns}]
set in_ports  [get_ports {m_axi_*ready m_axi_*valid m_axi_bid m_axi_bresp m_axi_rid m_axi_rdata m_axi_rresp m_axi_rlast vuart_rx_*}]
set out_ports [get_ports {m_axi_aw* m_axi_w* m_axi_ar* m_axi_bready m_axi_rready halted vuart_tx_* vuart_rx_pop dbg_probe*}]
if {[llength $in_ports]}  { set_input_delay  -clock clk $io_delay $in_ports }
if {[llength $out_ports]} { set_output_delay -clock clk $io_delay $out_ports }

# The abort_mask squash broadcast (RTL `(* max_fanout=64 *)` does not carry into OpenROAD).
set_max_fanout 64 [current_design]
