// scoreboard - ref_mode -  
// agents - הרכיבים שצריך לקבל את התוצאה
// sequencer - the operation you want to do
// driver - translate the operation to packet (transaction) 
// monitor - 
// register model - 
`define WIDTH 16

interface cesv;
    signed [`WIDTH-1:0] vector_a[8];
    signed [`WIDTH-1:0] vector_b[8];
endinterface

class VecDrv #(parameter WIDTH=16);
    // Parameters for the size of the vectors and their bit-width
    // int WIDTH;

    // Define two signed vectors of size n and width WIDTH
    rand logic signed [WIDTH-1:0] vector_a[];
    rand logic signed [WIDTH-1:0] vector_b[];
     
    // Constructor to initialize the size and width of the vectors
    function new(int size);
        vector_a = new[size];
        vector_b = new[size];
    endfunction

    // Randomization constraint to keep values within the range based on WIDTH
    constraint vector_values {
        foreach (vector_a[i]) vector_a[i] inside {[-(1 << (WIDTH-1)) : (1 << (WIDTH-1))-1]};
        foreach (vector_b[i]) vector_b[i] inside {[-(1 << (WIDTH-1)) : (1 << (WIDTH-1))-1]};
    }

    // Task to generate random values for both vectors
    task generate_vectors();
        if (!randomize()) begin
            $display("Error in randomizing vectors");
        end else begin
            $display("Randomized vectors generated successfully");
        end
    endtask
endclass //VecDrv

class Scoreboard;

    // Queue to store expected values
    int expected_sum_q[$];
    
    // Method to add expected values to the queue
    task add_expected_sum(VecDrv drv);
        int expected_sum;
        for (int i=0; i<$size(drv.vector_a); ++i) begin
            expected_sum += drv.vector_a[i] * drv.vector_b[i];
        end
        expected_sum_q.push_back(expected_sum);
    endtask
    
    // Method to compare expected and actual values
    task check(int actual_sum);
        int expected_sum;
        
        if (expected_sum_q.size() == 0) begin
            $display("Error: No expected sum to compare!");
            return;
        end

        // Retrieve the expected sum from the queue
        expected_sum = expected_sum_q.pop_front();
        
        // Compare the actual and expected sum
        if (expected_sum == actual_sum) begin
            $display("PASS: Actual sum (%0d) matches expected sum (%0d)", actual_sum, expected_sum);
        end else begin
            $display("FAIL: Actual sum (%0d) does not match expected sum (%0d)", actual_sum, expected_sum);
        end
    endtask

endclass

module tb_mac_vector_adder_tree();

    parameter N = 8;          // Vector size
    parameter WIDTH = `WIDTH;     // Bit width of each element in the vector

    reg clk;
    reg rst;
    reg signed [WIDTH-1:0] vector_A [0:N-1]; // Input vector A
    reg signed [WIDTH-1:0] vector_B [0:N-1]; // Input vector B
    wire signed [WIDTH*2-1:0] result;        // Output result
    
    // VecDrv vector_drv1;
    VecDrv #(WIDTH) fu;

    // Declare scoreboard instance
    Scoreboard scoreboard;
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

int i;
    // Test vector stimulus
    initial begin
        // Initialize clock and reset
        clk = 0;
        rst = 1;
        scoreboard = new();
        fu = new(8);
        #10 rst = 0;

        fu.generate_vectors();
        for ( i=0; i<8; ++i) begin
            vector_A[i] = fu.vector_a[i];
            vector_B[i] = fu.vector_b[i];
        end
        scoreboard.add_expected_sum(fu);
        @(posedge clk);
        @(posedge clk);
        scoreboard.check(result);
        // // Apply input vectors A and B
        // vector_A[0] = 16'sd5; vector_B[0] = 16'sd3;  // A[0] * B[0] = x += 5 * 3
        // vector_A[1] = 16'sd7; vector_B[1] = 16'sd2;  // A[1] * B[1] = x += 7 * 2
        // vector_A[2] = 16'sd4; vector_B[2] = 16'sd6;  // A[2] * B[2] = x += 4 * 6
        // vector_A[3] = 16'sd1; vector_B[3] = 16'sd8;  // A[3] * B[3] = x += 1 * 8
        // vector_A[4] = 16'sd9; vector_B[4] = 16'sd0;  // A[4] * B[4] = x += 9 * 0
        // vector_A[5] = 16'sd2; vector_B[5] = 16'sd5;  // A[5] * B[5] = x += 2 * 5
        // vector_A[6] = 16'sd3; vector_B[6] = 16'sd7;  // A[6] * B[6] = x += 3 * 7
        // vector_A[7] = 16'sd6; vector_B[7] = 16'sd4;  // A[7] * B[7] = x += 6 * 4

        // Wait for the result to be computed
        #100;

        // Display the final result
        $display("Final MAC result: %d", result);

        // End simulation
        #10 $finish;
    end

endmodule
