//INCDIRS
+INCDIR+$PULP_ENV/src/
+INCDIR+$PULP_ENV/src/soc/
+INCDIR+$PULP_ENV/src/soc/includes/
//+INCDIR+$PULP_ENV/src/soc/flash/
+INCDIR+$PULP_ENV/src/ips/ddr/altera_ddr_64bit
+INCDIR+$PULP_ENV/src/ips/pulpenix/src/fpga

//SUBCOMPONENTS
-f $PULP_ENV/src/ips/pulpenix/fpgnix.f

//FILES
$PULP_ENV/src/tb/fpgnix_tb.v
$PULP_ENV/src/tb/smart_uart_tb_non_r2d2.sv
$PULP_ENV/src/tb/py_spi_tb/sv/sock.c
$PULP_ENV/src/tb/py_spi_tb/sv/sock.sv
//$PULP_ENV/src/tb/py_spi_tb/sv/py_spi_tb.sv

// DDR dimm memory
$PULP_ENV/src/tb/ddr2_64bit_mem_model.v
$PULP_ENV/src/tb/ddr2_dimm.v


