module vec_mac #(
    parameter WIDTH = 16, N = 8, NUM_MACS = 2
) (
    input clk,
    input rst,
    input signed [NUM_MACS-1:0][WIDTH-1:0] [0:N-1] vector_A ,  // Input vector A
    input signed [NUM_MACS-1:0][WIDTH-1:0] [0:N-1] vector_B ,  // Input vector B
    output reg signed [WIDTH*2-1:0] result
);
// comb
mac_vector_adder_tree /*#(parameter WIDTH = 16, N = 8)*/ vmac1(
    .clk(clk),
    .rst(rst),
    .vector_A(vector_A[0]),  // Input vector A
    .vector_B(vector_B[0]),  // Input vector B
    .result()      // Output result
);

mac_vector_adder_tree /*#(parameter WIDTH = 16, N = 8)*/ vmac2(
    .clk(clk),
    .rst(rst),
    .vector_A(vector_A[1]),  // Input vector A
    .vector_B(vector_B[1]),  // Input vector B
    .result()      // Output result
);

// seq
// state machine, counter
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        
    end else begin
        
    end
end 

endmodule