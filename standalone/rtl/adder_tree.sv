module adder_tree_16 #(
    parameter WIDTH = 16  // Bit-width of each input
)(
    input wire signed [WIDTH-1:0] inputs [15:0], // 16-element vector input
    output wire signed [WIDTH-1:0] sum            // Output sum
);

    // Internal wires
    wire signed [WIDTH-1:0] level1 [7:0];
    wire signed [WIDTH-1:0] level2 [3:0];
    wire signed [WIDTH-1:0] level3 [1:0];
    wire signed [WIDTH-1:0] level4;

    // Level 1: Pairwise addition
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin: level1_gen
            assign level1[i] = inputs[2*i] + inputs[2*i + 1];
        end
    endgenerate

    // Level 2: Pairwise addition of level 1 results
    generate
        for (i = 0; i < 4; i = i + 1) begin: level2_gen
            assign level2[i] = level1[2*i] + level1[2*i + 1];
        end
    endgenerate

    // Level 3: Pairwise addition of level 2 results
    generate
        for (i = 0; i < 2; i = i + 1) begin: level3_gen
            assign level3[i] = level2[2*i] + level2[2*i + 1];
        end
    endgenerate

    // Level 4: Final addition of level 3 results
    assign level4 = level3[0] + level3[1];

    // Output: Sum of all levels
    assign sum = level4;

endmodule



// --------------------------
// 8-adder tree
// --------------------------
module adder_tree_8 #(
    parameter WIDTH = 16  // Bit-width of each input
)(
    input logic signed [2*WIDTH-1:0] inputs [0:7], // 8-element vector input
    output logic signed [2*WIDTH-1:0] sum            // Output sum
);

    // Internal wires
    wire signed [2*WIDTH-1:0] level1 [0:3]; // 4 intermediate sums from 8 inputs
    wire signed [2*WIDTH-1:0] level2 [0:1]; // 2 intermediate sums
    wire signed [2*WIDTH-1:0] level3;       // Final sum

    // Level 1: Pairwise addition
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin: level1_gen
            assign level1[i] = inputs[2*i] + inputs[2*i + 1];
        end
    endgenerate

    // Level 2: Pairwise addition of level 1 results
    generate
        for (i = 0; i < 2; i = i + 1) begin: level2_gen
            assign level2[i] = level1[2*i] + level1[2*i + 1];
        end
    endgenerate

    // Level 3: Final addition of level 2 results
    assign level3 = level2[0] + level2[1];

    // Output: Sum of all levels
    assign sum = level3;

endmodule