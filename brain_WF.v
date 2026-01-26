(* DONT_TOUCH = "true" *)
module Test1(
    input clk_50M, 
    input ir_in_F, ir_in_R, ir_in_L,
    input reset,        
    input EN_A_L, EN_A_R,
    input EN_B_L, EN_B_R,
    input echo1, echo2, echo3,
    output wire trig1, trig2, trig3,
    output op1, op2, op3, 
    output wire ENA, ENB,       
    output wire IN1, IN2, IN3, IN4 
);

// --- Internal Signals ---
reg start_out;     // Brain -> Robot: "Here is your command"
wire need_decision; // Robot -> Brain: "I need help!"
wire [1:0] event_in;  
reg [1:0] cmd_out;  
reg [15:0] u_turn_count;

// --- Instantiate Driver ---
motoring m1( 
    .clk_50M(clk_50M), 
    .reset(reset), 
    .ir_in_F(ir_in_F), .ir_in_R(ir_in_R), .ir_in_L(ir_in_L),
    .EN_A_L(EN_A_L), .EN_A_R(EN_A_R), .EN_B_L(EN_B_L), .EN_B_R(EN_B_R), 
    .echo1(echo1), .echo2(echo2), .echo3(echo3),    
    .cmd_in(cmd_out), 
    .start_in(start_out),
    .need_decision(need_decision),
    .event_out(event_in), 
    .trig1(trig1), .trig2(trig2), .trig3(trig3),
    .op1(op1), .op2(op2), .op3(op3), 
    .ENA(ENA), .ENB(ENB), 
    .IN1(IN1), .IN2(IN2), .IN3(IN3), .IN4(IN4)
);

// --- Brain Logic ---
reg [1:0] brain_state;
localparam WAIT_FOR_REQ = 2'd0;
localparam SEND_CMD     = 2'd1;

always @(posedge clk_50M or negedge reset) begin
    if(!reset) begin
        start_out <= 1'b0;
        cmd_out <= 2'b0;
        u_turn_count <= 0;
        brain_state <= WAIT_FOR_REQ;
    end
    else begin
        case(brain_state)
            
            // 1. Wait for Robot to ask for help
            WAIT_FOR_REQ: begin
                start_out <= 1'b0; // Keep command low
                
                if(need_decision == 1'b1) begin
                    brain_state <= SEND_CMD;
                    
                    // -- DECISION LOGIC --
                    case(event_in)
                        2'd3: begin // Dead End
                            cmd_out <= 2'd2; // U-Turn
                            u_turn_count <= u_turn_count + 1; 
                        end
                        2'd0: begin // Left Turn Found
                            if(u_turn_count == 4 || u_turn_count == 6)
                                cmd_out <= 2'd3; // Skip
                            else
                                cmd_out <= 2'd0; // Take Left
                        end
                        2'd1: begin // Right Turn Found
                            if(u_turn_count == 4 || u_turn_count == 6)
                                cmd_out <= 2'd3; // Skip
                            else
                                cmd_out <= 2'd1; // Take Right
                        end
                        2'd2: begin // Junction
                            if(u_turn_count == 1 || u_turn_count == 8)
                                cmd_out <= 2'd0; // Take Left
                            else
                                cmd_out <= 2'd1; // Take Right
                        end
                    endcase
                end
            end

            // 2. Send Command and Wait for Robot to Accept
            SEND_CMD: begin
                start_out <= 1'b1; // Raise Flag
                
                // If need_decision goes LOW, it means robot accepted the command
                if(need_decision == 1'b0) begin
                    start_out <= 1'b0; // Lower Flag
                    brain_state <= WAIT_FOR_REQ; // Go back to waiting
                end
            end
        endcase
    end
end
endmodule
