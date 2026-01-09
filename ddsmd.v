module Ultrasonic(
    input clk_50M, reset, echo1, echo2, echo3,
    output wire trig1, trig2, trig3,
    output op1, op2, op3,
    output wire [15:0] distance1, distance2, distance3
);

    localparam left = 2'b00, center = 2'b01, right = 2'b10;
    reg [23:0] counter;
    reg [1:0] sequence; 
    reg start_l, start_c, start_r;

    // --- FIXED INSTANTIATIONS (Added .trig connections) ---
    distance f(
        .clk_50M(clk_50M), .reset(reset), .echo_rx(echo1), 
        .start(start_l), .trig(trig1), // <--- WAS MISSING
        .op(op1), .distance_out(distance1)
    );
    
    distance r(
        .clk_50M(clk_50M), .reset(reset), .echo_rx(echo2), 
        .start(start_c), .trig(trig2), // <--- WAS MISSING
        .op(op2), .distance_out(distance2)
    );
    
    distance l(
        .clk_50M(clk_50M), .reset(reset), .echo_rx(echo3), 
        .start(start_r), .trig(trig3), // <--- WAS MISSING
        .op(op3), .distance_out(distance3)
    );

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            counter <= 0;
            sequence <= left;
            start_l <= 0; start_c <= 0; start_r <= 0;
        end 
        else begin
            // 1. Timer Logic
            if (counter >= 3000000) counter <= 0;   //3000000 => 60ms
            else counter <= counter + 1;

            // 2. Start Pulse Generation
            start_l <= 0; start_c <= 0; start_r <= 0; // Default low
            
            if (counter == 10) begin
                case(sequence)
                    left:   start_l <= 1;
                    center: start_c <= 1;
                    right:  start_r <= 1;
                endcase
            end

            // 3. Sequence Switching
            if (counter == 3000000) begin
                case(sequence)
                    left:   sequence <= center;
                    center: sequence <= right;
                    right:  sequence <= left;
                endcase
            end
        end
    end

endmodule
