## Copyright (C) 1991-2010 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, Altera MegaCore Function License 
## Agreement, or other applicable license agreement, including, 
## without limitation, that your use is for the sole purpose of 
## programming logic devices manufactured by Altera and sold by 
## Altera or its authorized distributors.  Please refer to the 
## applicable agreement for further details.

## VENDOR "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 9.1 Build 350 03/24/2010 Service Pack 2 SJ Web Edition"

## DATE "01/13/2019 22:08:56"

## 
## Device: Altera EPM570T100C5 Package TQFP100
## 

## 
## This Tcl script should be used for PrimeTime (Verilog) only
## 

## This file can be sourced in primetime

set report_default_significant_digits 3
set hierarchy_separator .

set quartus_root "f:/study/programs/quartus/quartus/"
set search_path [list . [format "%s%s" $quartus_root "eda/synopsys/primetime/lib"]  ]

set link_path [list *  maxii_io_lib.db maxii_asynch_lcell_lib.db  maxii_ufm_lib.db maxii_lcell_register_lib.db  alt_vtl.db]

read_verilog  maxii_all_pt.v 

##########################
## DESIGN ENTRY SECTION ##
##########################

read_verilog  read_encoder.vo
current_design read_encoder
link
## Set variable timing_propagate_single_condition_min_slew to false only for versions 2004.06 and earlier
regexp {([1-9][0-9][0-9][0-9]\.[0-9][0-9])} $sh_product_version dummy version
if { [string compare "2004.06" $version ] != -1 } {
   set timing_propagate_single_condition_min_slew false
}
set_operating_conditions -analysis_type single
read_sdf read_encoder_v.sdo

################################
## TIMING CONSTRAINTS SECTION ##
################################


## Start clock definition ##
# WARNING:  The required clock period is not set. The default value (100 ns) is used. 
create_clock -period 100.000 -waveform {0.000 50.000} [get_ports { CLK_20M } ] -name CLK_20M  
# WARNING:  The required clock period is not set. The default value (100 ns) is used. 
create_clock -period 100.000 -waveform {0.000 50.000} [get_ports { NADV } ] -name NADV  
# WARNING:  The required clock period is not set. The default value (100 ns) is used. 
create_clock -period 100.000 -waveform {0.000 50.000} [get_ports { NE1 } ] -name NE1  
# WARNING:  The required clock period is not set. The default value (100 ns) is used. 
create_clock -period 100.000 -waveform {0.000 50.000} [get_ports { NWE } ] -name NWE  
# WARNING:  The required clock period is not set. The default value (100 ns) is used. 
create_clock -period 100.000 -waveform {0.000 50.000} [get_ports { P_WRITE } ] -name P_WRITE  

set_propagated_clock [all_clocks]
set_clock_groups -asynchronous \
-group {CLK_20M} \
-group {NADV} \
-group {NE1} \
-group {NWE} \
-group {P_WRITE}
## End clock definition ##

## Start create collections ##
## End create collections ##

## Start global settings ##
## End global settings ##

## Start collection commands definition ##

## End collection commands definition ##

## Start individual pin commands definition ##
## End individual pin commands definition ##

## Start Output pin capacitance definition ##
set_load -pin_load 10 [get_ports { DATA[0] } ]
set_load -pin_load 10 [get_ports { DATA[10] } ]
set_load -pin_load 10 [get_ports { DATA[11] } ]
set_load -pin_load 10 [get_ports { DATA[12] } ]
set_load -pin_load 10 [get_ports { DATA[13] } ]
set_load -pin_load 10 [get_ports { DATA[14] } ]
set_load -pin_load 10 [get_ports { DATA[15] } ]
set_load -pin_load 10 [get_ports { DATA[1] } ]
set_load -pin_load 10 [get_ports { DATA[2] } ]
set_load -pin_load 10 [get_ports { DATA[3] } ]
set_load -pin_load 10 [get_ports { DATA[4] } ]
set_load -pin_load 10 [get_ports { DATA[5] } ]
set_load -pin_load 10 [get_ports { DATA[6] } ]
set_load -pin_load 10 [get_ports { DATA[7] } ]
set_load -pin_load 10 [get_ports { DATA[8] } ]
set_load -pin_load 10 [get_ports { DATA[9] } ]
set_load -pin_load 10 [get_ports { DIR1 } ]
set_load -pin_load 10 [get_ports { DIR2 } ]
set_load -pin_load 10 [get_ports { DIR3 } ]
set_load -pin_load 10 [get_ports { DIR4 } ]
set_load -pin_load 10 [get_ports { PULSE1 } ]
set_load -pin_load 10 [get_ports { PULSE2 } ]
set_load -pin_load 10 [get_ports { PULSE3 } ]
set_load -pin_load 10 [get_ports { PULSE4 } ]
set_load -pin_load 10 [get_ports { P_BUSY } ]
## End Output pin capacitance definition ##

## Start clock uncertainty definition ##
## End clock uncertainty definition ##

## Start Multicycle and Cut Path definition ##
## End Multicycle and Cut Path definition ##

## Destroy Collections ##

update_timing
