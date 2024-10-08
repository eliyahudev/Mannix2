module mac_vector_adder_tree #(parameter WIDTH = 16, N = 8)(
    input clk,
    input rst,
    input signed [0:N-1][WIDTH-1:0] vector_A ,  // Input vector A
    input signed [0:N-1][WIDTH-1:0] vector_B ,  // Input vector B
    output reg signed [WIDTH*2-1:0] result      // Output result
);

logic signed [0:N-1][WIDTH*2-1:0] mul ;  // Intermediate multiplication results
logic signed [0:(N-1)/2][WIDTH*2-1:0] mul2 ; // first layyer of adder


integer i;

// Multiplication stage
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        for (i = 0; i < N; i = i + 1)
            mul[i] <= 0;
    end else begin
        for (i = 0; i < N; i = i + 1)
            mul[i] <= vector_A[i] * vector_B[i];  // Element-wise multiplication
    end
end

adder_tree_8 ad8(
    .inputs(mul), // 8-element vector input
    .sum(result)     // Output sum
);

endmodule

