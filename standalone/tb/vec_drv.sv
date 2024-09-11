// class VecDrv #(int WIDTH=16);
//     // Parameters for the size of the vectors and their bit-width
//     int n;
//     // int WIDTH;

//     // Define two signed vectors of size n and width WIDTH
//     rand logic signed [WIDTH-1:0] vector_a[];
//     rand logic signed [WIDTH-1:0] vector_b[];

//     // Constructor to initialize the size and width of the vectors
//     function new(int size);
//         n = size;
//         vector_a = new[n];
//         vector_b = new[n];
//     endfunction

//     // Randomization constraint to keep values within the range based on WIDTH
//     constraint vector_values {
//         foreach (vector_a[i]) vector_a[i] inside {[-(1 << (WIDTH-1)) : (1 << (WIDTH-1))-1]};
//         foreach (vector_b[i]) vector_b[i] inside {[-(1 << (WIDTH-1)) : (1 << (WIDTH-1))-1]};
//     }

//     // Task to generate random values for both vectors
//     task generate_vectors();
//         if (!randomize()) begin
//             $display("Error in randomizing vectors");
//         end else begin
//             $display("Randomized vectors generated successfully");
//         end
//     endtask
// endclass //VecDrv
