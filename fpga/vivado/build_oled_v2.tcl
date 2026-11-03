# Auto-generated Husky-Plus + OLED-framebuffer build (Vivado 2026.1).
# Run from fpga/vivado:  vivado -mode batch -source build_oled.tcl
set_param general.maxThreads 8
set part xc7a100tftg256-3
create_project -force cwplus_oled_v2 ./cwplus_oled_v2 -part $part
set obj [current_project]
set_property target_language Verilog $obj
set_property "default_lib" "xil_defaultlib" $obj
set_property XPM_LIBRARIES {XPM_MEMORY XPM_FIFO XPM_CDC} $obj

set vfiles [list \
  "./../hdl/includes.v" \
  "./../hdl/setup.v" \
  "./../hdl/registers.v" \
  "./../hdl/clock_managment_advanced.v" \
  "./../hdl/fifo_top_husky.v" \
  "./../hdl/openadc_interface.v" \
  "./../hdl/reg_chipwhisperer.v" \
  "./../hdl/reg_clockglitch.v" \
  "./../hdl/reg_openadc.v" \
  "./../hdl/reg_openadc_adcfifo.v" \
  "./../hdl/trigger_resync.v" \
  "./../hdl/trigger_unit.v" \
  "./../hdl/usb_reg_main.v" \
  "./../hdl/cwhusky_top.v" \
  "./../hdl/reg_husky_adc.v" \
  "./../hdl/reg_mmcm_drp.v" \
  "./../hdl/xadc.v" \
  "./../hdl/slow_debug.v" \
  "./../hdl/clockglitch_a7.v" \
  "./../hdl/mmcm_phaseshift_interface.v" \
  "./../hdl/reg_la.v" \
  "./../hdl/userio.v" \
  "./../tracewhisperer/hdl/trace_top.v" \
  "./../tracewhisperer/hdl/reg_trace.v" \
  "./../tracewhisperer/hdl/fe_capture_trace.v" \
  "./../tracewhisperer/phywhisperer-common/hardware/hdl/fifo.v" \
  "./../tracewhisperer/phywhisperer-common/hardware/hdl/fe_capture_main.v" \
  "./../tracewhisperer/phywhisperer-common/hardware/hdl/reg_main.v" \
  "./../tracewhisperer/hdl/simple_trigger.v" \
  "./../hdl/sad.v" \
  "./../hdl/edge_trigger.v" \
  "./../hdl/trigger_pulse_cdc.v" \
  "./../fpga-common/hdl/cdc_simple.v" \
  "./../fpga-common/hdl/fifo_async.v" \
  "./../fpga-common/hdl/cdc_pulse.v" \
  "./../fpga-common/hdl/fifo_sync.v" \
  "./../fpga-common/hdl/cdc_bus.v" \
  "./../hdl/sad_x2_slowclock.v" \
  "./../hdl/trigger_chooser.v" \
  "./../hdl/trigger_sequencer.v" \
  "./../hdl/sad_x4_slowclock.v" \
  "./../hdl/esad.v" \
  "./../hdl/sad_group_trigger.v" \
  "./../fpga-common/hdl/uart_rx.v" \
  "./../hdl/hw_bb_trig.v" \
  "./../hdl/slow_fifo_wrapper.v" \
  "./../hdl/fast_fifo_wrapper.v" \
  "./../fpga-common/hdl/async_resetFFstyle2.v" \
  "./../fpga-common/hdl/crc_ss2.v" \
  "./../fpga-common/hdl/ss2.v" \
  "./../fpga-common/hdl/uart_core.v" \
  "./../hdl/reg_sca_display.v" \
  "./../hdl/spi_display_rx.v" \
  "./../hdl/ssd1306_slave.v" \
  "./../hdl/cdc_sync.v" \
  "./../hdl/reg_seph_terminal.v" \
  "./../hdl/iso7816_terminal.v" \
  "./../hdl/iso7816_uart.v" \
  "./../hdl/seph_mcu.v" \
  "./../hdl/trigger_cdc.v" \
  "./../hdl/sad_single_counter.v" \
  "./../hdl/sad_x2.v" \
  "./../tracewhisperer/phywhisperer-common/hardware/hdl/pw_trigger.v" \
]
add_files -norecurse $vfiles

set_property include_dirs [list \
  "[file normalize ../hdl]" \
  "[file normalize ../fpga-common/hdl]" \
  "[file normalize ../tracewhisperer/hdl]" \
  "[file normalize ../tracewhisperer/phywhisperer-common/hardware/hdl]" \
] [get_filesets sources_1]

set_property verilog_define {PLUS INTERVAL_MATCHING=1 ESAD} [get_filesets sources_1]

set xcifiles [list \
  "./cwhusky.srcs/sources_1/ip/fifo_generator_semipro/fifo_generator_semipro.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/fifo_generator_tiny/fifo_generator_tiny.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/xadc_wiz_0/xadc_wiz_0.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/adc_fast_fifo/adc_fast_fifo.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/fast_slower_fifo/fast_slower_fifo.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/tiny_adc_fast_fifo/tiny_adc_fast_fifo.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/usb_slow_fifo1/usb_slow_fifo1.xci" \
  "./cwhusky_plus.srcs/sources_1/ip/usb_slow_fifo2/usb_slow_fifo2.xci" \
  "./cwhusky.srcs/sources_1/ip/trigger_fifo/trigger_fifo.xci" \
  "./cwhusky.srcs/sources_1/ip/fifo_generator_0/fifo_generator_0.xci" \
]
add_files -norecurse $xcifiles

add_files -fileset constrs_1 -norecurse "./cwhusky_100.xdc"

set_property top cwhusky_top [get_filesets sources_1]
update_compile_order -fileset sources_1

puts "=== UPGRADING / GENERATING IP ==="
set ips [get_ips]
puts "IP in project: $ips"
if {[llength $ips]} {
  upgrade_ip -quiet $ips
  generate_target -quiet all $ips
}

puts "=== SYNTHESIS ==="
launch_runs synth_1 -jobs 8
wait_on_run synth_1
puts "SYNTH status: [get_property STATUS [get_runs synth_1]] / PROGRESS [get_property PROGRESS [get_runs synth_1]]"
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
  puts "SYNTH FAILED"; exit 1
}

puts "=== IMPLEMENTATION + BITSTREAM ==="
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
puts "IMPL status: [get_property STATUS [get_runs impl_1]] / PROGRESS [get_property PROGRESS [get_runs impl_1]]"
set bit [glob -nocomplain ./cwplus_oled_v2/cwplus_oled_v2.runs/impl_1/*.bit]
puts "BITSTREAM: $bit"
puts "=== DONE ==="
