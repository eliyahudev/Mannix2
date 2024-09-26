module vec_mac #(
    parameter WIDTH = 16, N = 8, NUM_MACS = 2
) (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [31:0] row_size,
    input signed [NUM_MACS-1:0][WIDTH-1:0] [0:N-1] vector_A ,  // Input vector A
    input signed [NUM_MACS-1:0][WIDTH-1:0] [0:N-1] vector_B ,  // Input vector B
    output reg signed [WIDTH*2-1:0] result,
    // output reg [31:0] counter ,  // Counter output for monitoring //ADD
    output reg done // Signal indicating process completion //ADD
);


localparam IDLE = 2'b00,
           SUM   = 2'b01,
           DONE   = 2'b11;
reg [31:0] counter, counter_r; 
reg [1:0] current_state, next_state;
reg signed [WIDTH*2-1:0] result_r;

wire signed [WIDTH*2-1:0] vmac1_result; // ADD
wire signed [WIDTH*2-1:0] vmac2_result; // ADD 

// comb
mac_vector_adder_tree /*#(parameter WIDTH = 16, N = 8)*/ vmac1(
    .clk(clk),
    .rst(rst),
    .vector_A(vector_A[0]),  // Input vector A
    .vector_B(vector_B[0]),  // Input vector B
    .result(vmac1_result)      // Output result
);

mac_vector_adder_tree /*#(parameter WIDTH = 16, N = 8)*/ vmac2(
    .clk(clk),
    .rst(rst),
    .vector_A(vector_A[1]),  // Input vector A
    .vector_B(vector_B[1]),  // Input vector B
    .result(vmac2_result)      // Output result
);


always @(*) begin
    case (current_state)
        IDLE: begin
            result = 0;
            counter = 0;
            done = 0;
            if(start) begin
                next_state = SUM; 
            end else begin
                next_state = IDLE;
            end 
        end
        SUM: begin
            if(counter == 0)begin
                result_r = 0;
            end
            #1
            // if (counter >= 80) begin
            //     done = 1;
            //     //counter = 0;  // Reset counter 
            //     next_state = DONE; 
            // end else begin
            result = result_r + (vmac1_result + vmac2_result);
            counter = counter_r + 16;
            done = (counter >= row_size) ? 1 : 0;
            next_state = ((counter >= row_size) && (start == 0)) ? IDLE : SUM;
            if((counter >= row_size) && (start == 1)) begin
                //result = 0;
                counter = 0;
                next_state = SUM;
            end
        end    
        // DONE: begin
        //     result = 0;
        //     counter = 0;
        //     done = 0;
        //     next_state = SUM; //should be discuse
        // end
        default: begin
            next_state = IDLE; // Default to IDLE
        end
    endcase
end


//seq
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        current_state <= IDLE; // Reset to the IDLE state
        result_r      <= 0;
        counter_r     <= 0;
    end else begin
        current_state <= next_state; // Transition to the next state
        result_r      <= result;
        counter_r     <= counter;
    end

end


// seq
// state machine, counter
//always @(posedge clk or negedge rst) begin
 //   if (rst) begin
 //       result <= 0;
  //      counter <= 1;
  //      done <= 0;
        
   // end else begin
        // Sum the two results each cycle
    //    result <= result + (vmac1_result + vmac2_result);
        
        // Check if the counter exceeds NumElement * TreeSize
     //   if (counter >= NUM_MACS * N) begin
      //      done <= 1;
      //      counter <= 1;  // Reset counter 
            

      //  end else begin
       //     counter <= counter + 1;
       //     done <= 0;
        
      //   end
   // end 
//end

endmodule