module tb_stream_multiply_add;

  reg clk;
  reg reset_n;
  reg signed [7:0] stream_a;
  reg signed [7:0] stream_b;
  wire signed [15:0] result;
  event push_stream;
  

 // Instantiate the stream_multiply_add module
  Stream_multiply_add stream_multiply_add (
    .clk(clk),
    .reset_n(reset_n),
    .stream_a(stream_a),
    .stream_b(stream_b),
    .result(result)
  );

  task automatic  insert_input(string file_a, string file_b, event push_stream, ref reg signed [7:0] stream_a, ref  reg signed [7:0] stream_b);
    int fd_a, fd_b;

    // initial begin
    stream_a = 0;
    stream_b = 0;

    fd_a = $fopen(file_a, "r");  
    fd_b = $fopen(file_b, "r");

    forever begin
      @(push_stream);
      void'($fscanf(fd_a, "%d", stream_a));
      void'($fscanf(fd_b, "%d", stream_b));      
    end

  // end
endtask 


  // Clock generation
always #5 clk = ~clk; // 10ns period clock

initial begin
  insert_input("../inputs/streama.txt", "../inputs/streamb.txt", push_stream, stream_a, stream_b);
end

  // Initialize and apply test vectors
initial begin
    // Initialize signals
  clk = 0;
  reset_n = 0;
    // Apply reset
    #5
    
repeat (50) begin
    #10
    ->push_stream;
    #10
    reset_n = 1; 
    ->push_stream;
    #10
    ->push_stream;
    #10
    ->push_stream;
    #20
    reset_n = 0;
end

    // Test vector 1
    // #10 stream_a = 8'h02; stream_b = 8'h03; // 2 * 3 = 6
    // #10 stream_a = 8'h01; stream_b = 8'h04; // 1 * 4 = 4; 6 + 4 = 10
    // #10 stream_a = -8'h01; stream_b = 8'h02; // -1 * 2 = -2; 10 - 2 = 8
    // #10 stream_a = 8'h00; stream_b = 8'h00; // 0 * 0 = 0; 8 + 0 = 8

    // Apply reset again
    //#30 reset_n = 0;
    //#30 reset_n = 1;
    
    // Test vector 2
    //#10 stream_a = 8'h03; stream_b = 8'h03; // 3 * 3 = 9
    //#10 stream_a = 8'h02; stream_b = -8'h02; // 2 * -2 = -4; 9 - 4 = 5
    //#10 stream_a = -8'h01; stream_b = -8'h01; // -1 * -1 = 1; 5 + 1 = 6

    // Stop simulation
  #50 $stop;
end





  // Monitor the signals
initial begin
  $monitor("Time = %0t : stream_a = %0d, stream_b = %0d, result = %0d", $time, stream_a, stream_b, result);
end
endmodule