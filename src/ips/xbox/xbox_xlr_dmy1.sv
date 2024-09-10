
module xbox_xlr_dmy1 #(parameter NUM_MEMS=1,LOG2_LINES_PER_MEM=4)  (

  // XBOX memories interface

   // System Clock and Reset
   input clk,
   input rst_n, // asserted when 0

   // Accelerator XBOX mastered memories interface

   output logic [NUM_MEMS-1:0][LOG2_LINES_PER_MEM-1:0] xlr_mem_addr,  //  address per memory instance
   output logic [NUM_MEMS-1:0] [7:0][31:0] xlr_mem_wdata, // 32 bytes write data interface per memory instance
   output logic [NUM_MEMS-1:0]      [31:0] xlr_mem_be,    // 32 byte-enable mask per data byte per instance.
   output logic [NUM_MEMS-1:0]             xlr_mem_rd,    // read signal per instance.
   output logic [NUM_MEMS-1:0]             xlr_mem_wr,    // write signal per instance.
   input        [NUM_MEMS-1:0] [7:0][31:0] xlr_mem_rdata, // 32 bytes read data interface per memory instance

   // Command Status Register Interface
   input  [31:0][31:0] host_regs,                   // regs accelerator write data, reflecting registers content as most recently written by SW over APB
   input        [31:0] host_regs_valid_pulse,       // reg written by host (APB) (one per register)
                                                    
   output [31:0][31:0] host_regs_data_out,          // regs accelerator write data,  this is what SW will read when accessing the register
                                                    // provided that the register specific host_regs_valid_out is asserted
   output       [31:0] host_regs_valid_out,         // reg accelerator (one per register)

   input [18:0]        trig_soc_xmem_wr_addr,       // optional trigger, XBOX memory address accessed by SOC (processor/apb)
   input               trig_soc_xmem_wr             // optional trigger, validate actual SOC xmem wr access   
) ;


// ILAAD: Inference-Driven Low-latency Autonomous AI Devices

logic [31:0][31:0] host_regs_data_out;          // regs accelerator write data,  this is what SW will read when accessing the register
                                                // provided that the register specific host_regs_valid_out is asserted
logic       [31:0] host_regs_valid_out;         // reg accelerator (one per register)

// TMP placeholder
assign xlr_mem_addr   = 0 ;
assign xlr_mem_wdata  = 0 ;
assign xlr_mem_be     = 0 ;
assign xlr_mem_rd     = 0 ;
assign xlr_mem_wr     = 0 ;


// assign host_regs_data_out  = 0 ; // Currently not in use for this accelerator                      
// assign host_regs_valid_out = 0 ; // Currently not in use for this accelerator
localparam dbg_addr = 3;
always @(posedge clk) begin
  if (host_regs_valid_pulse[dbg_addr])
    $display("a value %0d was writen to reg %0d", host_regs[dbg_addr], dbg_addr/*the register chose*/);
end
assign sw_go = (|host_regs[0]) & host_regs_valid_pulse[0];

   reg [3:0] count;          // 4-bit counter (0 to 10)
    reg [1:0] state;          // FSM state register

    // State encoding
    localparam IDLE    = 2'b00;
    localparam COUNT   = 2'b01;
    localparam FINISH  = 2'b10;

always_comb begin
  host_regs_data_out[0] = 32'd0;
  case (state)
    IDLE:
      host_regs_valid_out[0] = 0;
    COUNT:
      host_regs_valid_out[0] = 0;
    FINISH: begin
      host_regs_valid_out[0] = 1;
      host_regs_data_out[0]  = 32'd1;
    end
    default: begin
      host_regs_valid_out[0] = 0;
    end
  endcase   
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        count <= 4'd0;
    end else begin
        case (state)
            IDLE: begin
                // host_regs_valid_out <= 0;
                if (sw_go) begin
                    state <= COUNT;
                    count <= 4'd0;
                end
            end

            COUNT: begin
                // host_regs_valid_out <= 0;
                if (count < 4'd10) begin
                    count <= count + 1;
                end else begin
                    state <= FINISH;
                end
            end

            FINISH: begin
                $display($time," VERILOG MSG: a test passed") ;
                // host_regs_valid_out    <=  1;
                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

  // bf_exp exp(
  //   .clk(clk),      // inport
  //   .rst_n(rst_n),    // inport
  //   .start(host_regs[0]),    // inport
  //   .done_exp(host_regs_data_out[1]), // outport
  //   .done_sum(host_regs_data_out[2])  // outport
  // );

endmodule
