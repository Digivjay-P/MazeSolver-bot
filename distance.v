module distance(
    input clk_50M, reset, echo_rx,
    input start,
    output reg trig,
    output op,
    output wire [15:0] distance_out
);

    localparam WAIT=0, HIGH=1, LOW=2, IDLE=3;
    reg [1:0] state;
    reg [19:0] counter;
    reg [19:0] echo_counter;
    reg [15:0] distance_reg;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state <= IDLE;
            counter <= 0; echo_counter <= 0; distance_reg <= 0; trig <= 0;
        end 
        else begin
            case (state)
                IDLE: begin
                    trig <= 0; 
                    echo_counter <= 0; 
                    counter <= 0;
                    if (start) state <= WAIT;
                end

                WAIT: begin
                    counter <= counter + 1;
                    if (counter > 50) begin
                        trig <= 1;
                        state <= HIGH;
                    end
                end

                HIGH: begin
                    counter <= counter + 1;
                    if (counter > 550) begin
                        trig <= 0;
                        state <= LOW;
                    end
                end

                LOW: begin
                    counter <= counter + 1;

                    // 1. Safety Timeout (e.g., 30ms = 1.5M cycles)
                    // If echo gets stuck high or low for too long, reset.
                    if (counter > 1500000) begin
                        state <= IDLE;
                    end
                    // 2. Measure Echo
                    else if (echo_rx == 1) begin
                        echo_counter <= echo_counter + 1;
                    end
                    // 3. Pulse Ended
                    else if (echo_counter > 0) begin
                        state <= IDLE;
                        // Filter: Only update if distance is valid (< 400cm)
                        // 400cm * 10000 / 34 = ~117,647 counts
                        if (echo_counter < 118000) begin
                             distance_reg <= (echo_counter * 34) / 10000;
                        end
                        echo_counter <= 0;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    assign distance_out = distance_reg;
    assign op = (distance_out < 71 && distance_out > 0) ? 1'b1 : 1'b0;

endmodule
