// scoreboard - ref_mode -  
// agents - הרכיבים שצריך לקבל את התוצאה
// sequencer - the operation you want to do
// driver - translate the operation to packet (transaction) 
// monitor - 
// register model - 

module tb_mac_vector_adder_tree();

    parameter N = 8;          // Vector size
    parameter WIDTH = 16;     // Bit width of each element in the vector

    reg clk;
    reg rst;
    reg signed [WIDTH-1:0] vector_A [0:N-1]; // Input vector A
    reg signed [WIDTH-1:0] vector_B [0:N-1]; // Input vector B
    wire signed [WIDTH*2-1:0] result;        // Output result

    // Instantiate the MAC vector adder tree
    mac_vector_adder_tree #(N, WIDTH) mac_inst (
        .clk(clk),
        .rst(rst),
        .vector_A(vector_A),
        .vector_B(vector_B),
        .result(result)
    );

    // Clock generation
    always begin
        #5 clk = ~clk;  // 10 time unit clock period
    end

    // Test vector stimulus
    initial begin
        // Initialize clock and reset
        clk = 0;
        rst = 1;
        #10 rst = 0;

        // Apply input vectors A and B
        vector_A[0] = 16'sd5; vector_B[0] = 16'sd3;  // A[0] * B[0] = x += 5 * 3
        vector_A[1] = 16'sd7; vector_B[1] = 16'sd2;  // A[1] * B[1] = x += 7 * 2
        vector_A[2] = 16'sd4; vector_B[2] = 16'sd6;  // A[2] * B[2] = x += 4 * 6
        vector_A[3] = 16'sd1; vector_B[3] = 16'sd8;  // A[3] * B[3] = x += 1 * 8
        vector_A[4] = 16'sd9; vector_B[4] = 16'sd0;  // A[4] * B[4] = x += 9 * 0
        vector_A[5] = 16'sd2; vector_B[5] = 16'sd5;  // A[5] * B[5] = x += 2 * 5
        vector_A[6] = 16'sd3; vector_B[6] = 16'sd7;  // A[6] * B[6] = x += 3 * 7
        vector_A[7] = 16'sd6; vector_B[7] = 16'sd4;  // A[7] * B[7] = x += 6 * 4

        // Wait for the result to be computed
        #100;

        // Display the final result
        $display("Final MAC result: %d", result);

        // End simulation
        #10 $finish;
    end

endmodule
