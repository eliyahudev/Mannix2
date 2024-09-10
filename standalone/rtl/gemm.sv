module mac_vector_adder_tree #(parameter N = 8, WIDTH = 16)(
    input clk,
    input rst,
    input signed [WIDTH-1:0] vector_A [0:N-1],  // Input vector A
    input signed [WIDTH-1:0] vector_B [0:N-1],  // Input vector B
    output reg signed [WIDTH*2-1:0] result      // Output result
);

logic signed [WIDTH*2-1:0] mul [0:N-1];  // Intermediate multiplication results
logic signed [WIDTH*2-1:0] sum [0:N/2-1]; // Adder tree summation

integer i;

// Multiplication stage
always @(posedge clk or posedge rst) begin
    if (rst) begin
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

