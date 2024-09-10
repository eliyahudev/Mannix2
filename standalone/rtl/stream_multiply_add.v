module Stream_multiply_add (
  input clk,
  input reset_n,
  input signed [7:0] stream_a,
  input signed [7:0] stream_b,
  output signed [15:0] result
);

  // Registers to hold the previous clock cycle's data
  reg signed [7:0] stream_a_reg;
  reg signed [7:0] stream_b_reg;
  reg signed [15:0] prev_result;

  // Calculate the multiplication result
  wire signed [15:0] mult_result = stream_a_reg * stream_b_reg;

  // Calculate the sum and store in the output register
  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      prev_result <= 0;
    end else begin
      prev_result <= mult_result + prev_result;
    end
  end

  // Update the input registers on every clock cycle
  always @(posedge clk) begin
    stream_a_reg <= stream_a;
    stream_b_reg <= stream_b;
  end

  // Assign the current result to the output
  assign result = prev_result;

endmodule


