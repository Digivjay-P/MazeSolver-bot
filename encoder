`timescale 1ns / 1ps

module encoder(
    input wire clk_50M,
    input wire reset,
(* MARK_DEBUG = "true" *)   input wire hwa,           // Single Hall effect sensor
(* MARK_DEBUG = "true" *)    output reg [19:0] counter // Total position (ticks)
);

    // 1. Two-Stage Synchronizer to prevent metastability
    reg hwa_sync_1;
    reg hwa_sync_2;
    
    // 2. Edge Detection register
    reg hwa_prev;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            hwa_sync_1 <= 1'b0;
            hwa_sync_2 <= 1'b0;
            hwa_prev   <= 1'b0;
            counter    <= 20'd0;
        end 
        else begin
            // Shift the signal through the synchronizer
            hwa_sync_1 <= hwa;
            hwa_sync_2 <= hwa_sync_1;
            
            // Store the previous synchronized state to detect edges
            hwa_prev <= hwa_sync_2;
            
            // RISING EDGE DETECTION: 
            // If it was LOW last clock, and is HIGH this clock, it just transitioned!
            if (hwa_sync_2 == 1'b1 && hwa_prev == 1'b0) begin
                counter <= counter + 1;
            end
        end
    end

endmodule
