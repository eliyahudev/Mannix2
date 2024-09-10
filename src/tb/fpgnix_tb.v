
`timescale 1ps/1ps
module fpgnix_tb();

reg clk_ref;
reg rst_n;

// CLOCK
`ifdef ALTERA
    localparam REF_CLK_PERIOD = 40;
`else
    localparam REF_CLK_PERIOD = 100;
`endif
localparam REF_CLK_PERIOD_PS = (REF_CLK_PERIOD*1000);

parameter time TIMEOUT = 1000000000;

initial begin
    clk_ref = 1'b0;
    forever #(REF_CLK_PERIOD_PS/2) clk_ref = ~clk_ref;
end

// RESET
initial begin
    rst_n = 1'b0;
    #1
    @(posedge clk_ref)
    rst_n = 1'b1;
end

wire uart_tx;
logic uart_rx;
logic [3:0] led;
//jtag
wire VTref;
wire RTCK;
wire TDO;
wire TRSTn;
wire TDI;
wire TMS;
wire TCK;
wire nSRST;
assign nSRST = 1'b1;


// DDR memory instance
`ifdef DDR_INTRFC
`ifndef PSD_DDR

	wire [1:0]	mem_odt;
	wire [1:0]	mem_cs_n;
	wire [1:0]	mem_cke;
	wire [13:0]	mem_addr;
	wire [2:0]	mem_ba;
	wire 	    mem_ras_n;
	wire 	    mem_cas_n;
	wire 	    mem_we_n;
	wire [7:0]	mem_dm;
	wire [1:0]	mem_clk;
	wire [1:0]	mem_clk_n;
	wire [63:0]	mem_dq;
	wire [7:0]	mem_dqs;
	wire 	    ddr_led;
    
    wire pad_spis_clk;
    wire pad_spis_cs;
    wire pad_spis_di;
    wire pad_spis_do;     
    
    
ddr2_dimm ddr2_dimm (     // ddr2 actual DIMM
	   
	   .mem_odt    ( mem_odt   ),  // output
	   .mem_cs_n   ( mem_cs_n  ),  // output
	   .mem_cke    ( mem_cke   ),  // output
	   .mem_addr   ( mem_addr  ),  // output
	   .mem_ba     ( mem_ba    ),  // output
	   .mem_ras_n  ( mem_ras_n ),  // output
	   .mem_cas_n  ( mem_cas_n ),  // output
	   .mem_we_n   ( mem_we_n  ),  // output
	   .mem_dm     ( mem_dm    ),  // output
                               
	   .mem_clk    ( mem_clk   ),  // inout
	   .mem_clk_n  ( mem_clk_n ),  // inout
	   .mem_dq     ( mem_dq    ),  // inout
	   .mem_dqs    ( mem_dqs   )   // inout	         
	);
        

`endif // DDR_INTRFC
`endif // ndef PSD_DDR


fpgnix fpgnix (
              // Outputs
              .uart_tx                  (uart_tx),
              .led                      (led[3:0]),
              .VTref                    (VTref),
              .RTCK                     (RTCK),
              .TDO                      (TDO),
              // Inputs
              .sys_rst                  (rst_n), 
              .altera_clk25mhz          (clk_ref),
              .uart_rx                  (uart_rx),
              .nTRST                    (TRSTn),
              .TDI                      (TDI),
              .TMS                      (TMS),
              .TCK                      (TCK),
              .nSRST                    (nSRST),
              
              `ifdef DDR_INTRFC
              `ifndef PSD_DDR             
               
              .mem_odt    ( mem_odt   ),  // output [1:0]	
	          .mem_cs_n   ( mem_cs_n  ),  // output [1:0]	
	          .mem_cke    ( mem_cke   ),  // output [1:0]	
	          .mem_addr   ( mem_addr  ),  // output [13:0]	
	          .mem_ba     ( mem_ba    ),  // output [2:0]	
	          .mem_ras_n  ( mem_ras_n ),  // output		    
	          .mem_cas_n  ( mem_cas_n ),  // output		    
	          .mem_we_n   ( mem_we_n  ),  // output		    
	          .mem_dm     ( mem_dm    ),  // output [7:0]	
	          .mem_clk    ( mem_clk   ),  // inout  [1:0]	
	          .mem_clk_n  ( mem_clk_n ),  // inout  [1:0]	
	          .mem_dq     ( mem_dq    ),  // inout  [63:0]	
	          .mem_dqs    ( mem_dqs   ),  // inout  [7:0]
                           
              `endif // DDR_INTRFC
              `endif // ndef PSD_DDR

              .pad_spis_clk        (pad_spis_clk),
              .pad_spis_cs         (pad_spis_cs),
              .pad_spis_di         (pad_spis_di),
              .pad_spis_do         (pad_spis_do)              
              
            );


`ifdef ENABLE_SPI_PY_TB
py_spi_tb py_spi_tb (
    .spi_sdi(pad_spis_do),
    .spi_csn(pad_spis_cs),    
    .spi_sck(pad_spis_clk),    
    .spi_sdo(pad_spis_di)    
) ;
`else

assign pad_spis_clk = 1'b0 ; 
assign pad_spis_cs  = 1'b0 ;
assign pad_spis_di  = 1'b0 ;
    
`endif



               
// ======
//  UART
// ======
`define R2D2_OPTION (0)
`ifdef R2D2_TELNET
    `define R2D2_OPTION (1)
`endif

`ifdef R2D2_PYSHELL
    `define R2D2_OPTION (2)
`endif

`ifdef R2D2_TXONLY
    `define R2D2_OPTION (3)
`endif

`ifdef R2D2_JTAG
    `define R2D2_OPTION (4)
`endif

wire clk_sys;
assign clk_sys = fpgnix.vqm_msystem_wrap.msystem.clk_sys;


`ifdef SMART_UART_TB_NON_R2D2
  
// Smart uart tb including file access from running C code but without R2D2 support
// Also compliant with FPGA/DDP pyshell terminal and file access SW interface.
// This means same compiled code images with file access can run both in this mode and in the FPGA.
// Alternatively uart_vip currently also support most to all of above

uart_bus #(

    .BAUDRATE(`BAUD_RATE),    
    .PARITY_EN(0),
    .NUM_SELECTABLE_UARTS(1) 
  )    
  i_uart_bus (  
    .uart_rx(uart_tx),
    .uart_tx(uart_rx),
    .rx_en('1)
  );


`else


uart_vip
  #(
    .BAUD_RATE(`BAUD_RATE),
    .PARITY_EN(0),
    .R2D2_OPTION(`R2D2_OPTION)

    `ifdef USE_SOCKET
        , .USE_SOCKET(1)
    `endif

  )
  uart_vip
  (
    .rx         ( uart_tx ),
    .tx         ( uart_rx ),
    .rx_en      ( 1'b1    ),
    .clk        ( clk_sys )
  );
  
`endif  
  
  pulldown uart_pulldown (uart_tx);

// jtag calls from dpi
// based on riscv/tb/dm/tb_test_env.sv
`ifdef JTAG_BITBANG
logic sim_jtag_exit;
logic sim_jtag_enable;
initial begin
    sim_jtag_enable = 1'b0;
    #1000
    sim_jtag_enable = 1'b1;
end
localparam OPENOCD_PORT=8989;

SimJTAG #(
        .TICK_DELAY (300),
        .PORT(OPENOCD_PORT))
        i_sim_jtag (
                    //.clock                (clk_sys),
                    .reset                (~rst_n),
                    .enable               (sim_jtag_enable),
                    .init_done            (rst_n),
                    .jtag_TCK             (TCK),
                    .jtag_TMS             (TMS),
                    .jtag_TDI             (TDI),
                    .jtag_TRSTn           (TRSTn),
                    .jtag_TDO_data        (TDO),
                    .jtag_TDO_driven      (1'b1),
                    .exit                 (sim_jtag_exit));

    always @(*) begin : jtag_exit_handler
        if (sim_jtag_exit) begin
            sim_jtag_enable = 1'b0;
            $stop(2); // allow user to continue running the simulation after jtag disconnect
        end
    end
`else //!JTAG_BITBANG
    assign TCK   = 1'b0;
    assign TRSTn = 1'b0;
    assign TMS   = 1'b0;
    assign TDI   = 1'b0;
`endif //JTAG_BITBANG

endmodule
