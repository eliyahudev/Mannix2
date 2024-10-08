
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
// assign xlr_mem_addr   = 0 ;
// assign xlr_mem_wdata  = 0 ;
// assign xlr_mem_be     = 0 ;
// assign xlr_mem_rd     = 0 ;
// assign xlr_mem_wr     = 0 ;

// parameters
parameter N = 8;          // Vector size
parameter WIDTH = 16;     // Bit width of each element in the vector
parameter NUM_MACS = 2;


// assign host_regs_data_out  = 0 ; // Currently not in use for this accelerator                      
// assign host_regs_valid_out = 0 ; // Currently not in use for this accelerator
localparam dbg_addr = 3;
always @(posedge clk) begin
  if (host_regs_valid_pulse[dbg_addr])
    $display("a value %0d was writen to reg %0d", host_regs[dbg_addr], dbg_addr/*the register chose*/);
end
assign sw_go = (|host_regs[8]) & host_regs_valid_pulse[8];

   reg [31:0] count;          // 4-bit counter (0 to 10)
    reg [1:0] state;          // FSM state register
    reg [31:0]num_row ;
    wire row_done;
    // State encoding
    localparam IDLE    = 2'b00;
    localparam COUNT   = 2'b01;
    localparam FINISH  = 2'b10;

// TODO connect to the mac block and delete this line
// this line acting like every clock cycle one row has finished
assign row_done = 1;  

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        count <= 4'd0;
    end else begin
        case (state)
            IDLE: begin
                // host_regs_valid_out <= 0;
                host_regs_data_out[0] <= host_regs_data_out[0];
                host_regs_valid_out[0] = host_regs_valid_out[0];
                if (sw_go) begin
                    $display("!@#$^&*(&$##@@#$$$$$####@@#$$$#@@#$^&(()))()()(&^^!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!):");
                    $display("!@#$^&*(&$##@@#$$$$$####@@#$$$#@@#$^&(()))()()(&^^!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!):");
                    $display("!@#$^&*(&$##@@#$$$$$####@@#$$$#@@#$^&(()))()()(&^^!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!):");
                    $display("!@#$^&*(&$##@@#$$$$$####@@#$$$#@@#$^&(()))()()(&^^!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!):");
                    $display("host_regs_valid_out[8]: %d", host_regs_valid_out[8]);
                    $display("host_regs_valid_out[1]: %d", host_regs_valid_out[1]);
                    $display("host_regs_data_out[1]: %d", host_regs_data_out[1]);
                    
                    $display("row size: %d; col size: %d", host_regs[3], host_regs[2]);
                    state <= COUNT;
                    count <= 4'd0;
                end
            end

            COUNT: begin
                host_regs_valid_out[0] = 0;
                host_regs_data_out[0] <= 32'd0;
                // host_regs_valid_out <= 0;
                $display("row size: %d; col size: %d", host_regs[3], host_regs[2]);
                num_row = host_regs[3];
                if (count < host_regs[3]) begin //Oz&Eliyahu : work only when host_regs[3] =10.....
                    count <= count + 1;
                     $display("counter: %d", count);
                end else begin
                    state <= FINISH;
                    xlr_mem_addr[0] <= 0;
                    xlr_mem_be[0][0] <= 1;
                    xlr_mem_be[0][1] <= 1;
                    xlr_mem_rd[0] <= 1;
                end
            end

            FINISH: begin
                $display($time," VERILOG MSG: a test passed") ;
                xlr_mem_addr <= 0;
                // host_regs_valid_out    <=  1;
                host_regs_valid_out[0] <= 1;
                host_regs_data_out[0]  <= 32'd1;
                state <= IDLE;
                $display("xlr_mem_rdata[0][0] = %d",xlr_mem_rdata[0][0]);
                $display("xlr_mem_rdata[0][1] = %d",xlr_mem_rdata[0][1]);
            end

            default: state <= IDLE;
        endcase
    end
end

vec_mac #(N, WIDTH, NUM_MACS) mac_inst (  //changeADD
        .clk(clk),
        .rst(rst),
        // .start(start),
        .row_size(32)
        // .vector_A(vector_A),
        // .vector_B(vector_B),
        // .result(result),
        // .counter(counter) ,  //ADD
        // .done(done)  //ADD
    );

// always_comb begin
//   case (state)
//     IDLE:
//       host_regs_valid_out[1] = 0;
//     COUNT:
//       host_regs_valid_out[1] = 0;
//     FINISH: begin
//       host_regs_valid_out[1] = 1;
//       host_regs_data_out[1]  = 32'd1;
//     end
//     default: begin
//       host_regs_valid_out[1] = 0;
//     end
//   endcase   
// end




// vec_mac #(
//     parameter WIDTH = 16, N = 8, NUM_MACS = 2
// ) (
//     .clk(clk),
//     .rst(rst),
//     .start(start),
//     .row_size(),
//     .vector_A() ,  // Input vector A
//     .vector_B() ,  // Input vector B
//     .result(),
//     .done(row_done) // Signal indicating process completion //ADD
// );

  // Mac mac(
  //   .clk(clk),      // inport
  //   .rst_n(rst_n),    // inport
  //   .start(host_regs[0]),    // inport
  //   .done_exp(host_regs_data_out[1]), // outport
  //   .done_sum(host_regs_data_out[2])  // outport
  // );

endmodule
