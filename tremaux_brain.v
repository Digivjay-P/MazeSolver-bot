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

/*

//---------------------------------
/*
FOR EVENT IN:
0 : LEFT
1 : RIGHT
2 : JUNCTION
3 : DEADNED


FOR CMD_OUT :
0 : LEFT
1 : RIGHT
2 : U TURN
3 : DONT TAKE THE TURN THAT WAS JUST DETECTED, go straight


START POS   : 4,8 (Testbench pos 76)
EXIT POS    : 4,0 (Testbench pos 4)
DEADENDS    : 9

*/

//--------------------------------------
//------- Distance----------------------
(* keep *) wire [15:0] dL; // Left Distance
(* keep *) wire [15:0] dF; // Front Distance
(* keep *) wire [15:0] dR; // Right Distance

// --- Internal Signals ---
reg start_out;     // Brain -> Robot: "Here is your command"
wire need_decision; // Robot -> Brain: "I need help!"
wire [1:0] event_in;  
reg [1:0] cmd_out; 
wire in_follow; //to indicate bot is in follow state 

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
    .IN1(IN1), .IN2(IN2), .IN3(IN3), .IN4(IN4),
    .dl(dL), . df(dF), .dr(dR),
    .counter_R (encoder_counter_R_current),
	 .in_follow(in_follow)
    
);


localparam N=3'd0, E=3'd1, S=3'd2, W=3'd3;
localparam cmd_forward = 2'd3, cmd_left = 2'd0, cmd_right = 2'd1, cmd_uturn = 2'd2;

// --- State Registers ---
reg [1:0] state;          // 00: deadend, 01: path, 10: junction
reg [2:0] current_dir;    // N, E, S, W
reg [1:0] mark_mem [0:80][0:3]; // 81 cells*4 directions (stores 0,1,2)

// --- Next-State Logic Wires/Regs ---
reg [1:0] next_state;
reg [2:0] next_move;
reg [2:0] next_dir;
reg [7:0] idx;

// --- Helper regs for combinational block ---
reg [2:0] absL, absF, absR, absU;  // Absolute directions
reg openL, openF, openR;           // Open paths
reg [1:0] markL, markF, markR;     // Marker values
//---------------------------------------------
localparam open_distance = 400; //40cm
localparam cell_threshold = 3000; //encoder value for cell change 


//-------------------------------------------------
reg [19:0] R_ref;
wire [19:0] R_diff; 
wire  [19:0] encoder_counter_R_current;     //reads encoder current value from motoring module 

assign R_diff = (encoder_counter_R_current > R_ref) ? encoder_counter_R_current - R_ref : R_ref - encoder_counter_R_current;
//- --- -  ---- ----- --------------------x-----

reg [1:0] brain_state;
localparam WAIT_FOR_REQ = 2'd0;
localparam DECIDE = 2'd1;
localparam SEND_CMD = 2'd2;

// --- Helper variables ---
integer i, j;
reg [7:0] current_cell, next_cell;
reg signed [2:0] relL, relF, relR, relU;

integer delta [0:3];

// --- Helper Function ---
function automatic [2:0] new_dir(input [2:0] current_dir, input signed [2:0] rel);
begin
    new_dir = (current_dir + rel + 3'd4) % 3'd4; // Modulo 4 for wrap-around
end
endfunction

reg [2:0] min_dir;   
reg [7:0] min_value;

always @(*) begin
    // Compute absolute directions for sensors (wrap-around 0–3)
    absL = new_dir(current_dir, relL);
    absF = new_dir(current_dir, relF);
    absR = new_dir(current_dir, relR);
    absU = new_dir(current_dir, relU);

    openL = (dL > open_distance) ? 1'b1 : 1'b0 ;
    openR = (dR > open_distance) ? 1'b1 : 1'b0 ;
    openF = (dF > open_distance) ? 1'b1 : 1'b0 ;
end

always @(posedge clk_50M or negedge reset)begin
if(!reset)begin
    // Deltas for (N, E, S, W)
    relL = -3'sd1; // Left
    relF =  3'sd0; // Forward
    relR =  3'sd1; // Right
    relU =  3'sd2; // U-Turn
    delta[0] = -8'sd9;  // N 
    delta[1] =  8'sd1;  // E
    delta[2] =  8'sd9;  // S
    delta[3] = -8'sd1;  // W
    current_cell = 7'd76;
    current_dir = N;
    brain_state <= WAIT_FOR_REQ;
    state <= 2'b01;
    current_dir  <= N;
    current_cell <= 7'd76;
    x            <= 8'd4;
    y            <= 8'd8;
	 idx <= 0;
	 R_ref <= 0; 
    for (i = 0; i < 81; i = i + 1)
        for (j = 0; j < 4; j = j + 1)
        mark_mem[i][j] <= 2'b00;
end
else begin
case(brain_state)
WAIT_FOR_REQ : begin
    
    if(need_decision == 1'b1)begin
    // Get marker values for open paths
    markL = openL ? mark_mem[current_cell][absL] : 2'd3; 
    markF = openF ? mark_mem[current_cell][absF] : 2'd3; 
    markR = openR ? mark_mem[current_cell][absR] : 2'd3; 

    // --- Compute min_dir here ---
    min_value = markF;
    min_dir   = 3'd000;
    if (markR < min_value) begin
        min_value = markR;
        min_dir   = 3'd001;
        brain_state <= DECIDE;
    end
    if (markL < min_value) begin
        min_value = markL;
        min_dir   = 3'd010;
        brain_state <= DECIDE;

    end
    if (markF == markL) begin
        min_dir = 3'd011;
        brain_state <= DECIDE;
    end
    if (markF == markR) begin
        min_dir = 3'd100;
        brain_state <= DECIDE;
    end
	 if(markL == markR) begin
		min_dir = 3'd111;
        brain_state <= DECIDE;
	 end
    if ((markF == markR) && (markF == markL)) begin
        min_dir = 3'd101;
        brain_state <= DECIDE;
    end

    // ----------- State Classification ---------
    if (event_in == 2'd3)
        next_state = 2'b00; // deadend
    else if (event_in == 2'd0 || event_in == 2'd1)
        next_state = 2'b01; // left or right turn  
    else if(event_in == 2'd2)
        next_state = 2'b10; // junction
end
else if(in_follow == 1'b1) begin //when need help is not high bot is in s follow state
    if(R_diff > cell_threshold) begin // update this when bot has moved to next cell
        R_ref <= encoder_counter_R_current ;
        current_cell <= $signed(current_cell) + delta[current_dir];
        x <= ($signed(current_cell) + delta[current_dir]) % 9;
        y <= ($signed(current_cell) + delta[current_dir]) / 9;
		  if (mark_mem[current_cell][current_dir] < 2'd2) begin
             mark_mem[current_cell][current_dir] <= mark_mem[current_cell][current_dir] + 2'd1;
        end
		  if (mark_mem[$signed(current_cell) + delta[current_dir]][new_dir(current_dir, relU)] < 2'd2) begin
             mark_mem[$signed(current_cell) + delta[current_dir]][new_dir(current_dir, relU)] <= mark_mem[$signed(current_cell) + delta[current_dir]][new_dir(current_dir, relU)] + 2'd1;
        end
    end
end
end
DECIDE : begin
//-------------------------------------
// --- State-Based Action Logic ---
case (next_state)

    // --- Deadend ---
    2'b00 : begin
        cmd_out = cmd_uturn;
        next_dir  = absU;
        next_cell = $signed(current_cell) + delta[next_dir];
        brain_state <= SEND_CMD;
        end
        
    // --- Path ---
    2'b01: begin
    if (event_in == 2'd0) begin  //left turn was detected
        cmd_out = cmd_left;
        next_dir  = absL;
        next_cell = $signed(current_cell) + delta[next_dir];
        brain_state <= SEND_CMD;
        end
    else if (event_in == 2'd1) begin  //right turn was detected take right 
        cmd_out = cmd_right;
        next_dir  = absR;
        next_cell = $signed(current_cell) + delta[next_dir];
        brain_state <= SEND_CMD;
        end
    end
    // --- Junction ---
        2'b10: begin
        case (min_dir)
        3'd000: begin
            cmd_out = cmd_forward;
            next_dir  = absF;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
        3'd001: begin
            cmd_out = cmd_right;
            next_dir  = absR;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
        3'd010: begin
            cmd_out = cmd_left;
            next_dir  = absL;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
        3'd011: begin
		    if((current_cell != 56 && idx !=48) && (current_cell != 58 && idx !=56)) begin
                cmd_out = cmd_left;
                next_dir  = absL;
                next_cell = $signed(current_cell) + delta[next_dir];
                brain_state <= SEND_CMD;
			end
			else begin
				cmd_out = cmd_forward;
                next_dir  = absF;
                next_cell = $signed(current_cell) + delta[next_dir];
                brain_state <= SEND_CMD;
			end								
        end
        3'd100: begin
            cmd_out = cmd_forward;
            next_dir  = absF;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
        3'd101: begin
            cmd_out = cmd_left;
            next_dir  = absL;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
		3'd111: begin
            cmd_out = cmd_left;
            next_dir  = absL;
            next_cell = $signed(current_cell) + delta[next_dir];
            brain_state <= SEND_CMD;
        end
        endcase
        end
    endcase
end
SEND_CMD : begin
    // 1. Raise flag to the Motor Driver
    start_out <= 1'b1; 

    // 2. Update Memory and Commit State ONCE when the robot accepts the command
    if(need_decision == 1'b0) begin
        start_out <= 1'b0; 
        R_ref <= encoder_counter_R_current;
        // --- Tremaux Memory Update ---
        // Mark the path we are taking out of the current cell
        if (mark_mem[current_cell][next_dir] < 2'd2)
            mark_mem[current_cell][next_dir] <= mark_mem[current_cell][next_dir] + 2'd1;

        // Mark the path we use to enter the NEXT cell (the U-turn/opposite direction)
        if (mark_mem[next_cell][new_dir(next_dir, relU)] < 2'd2)
            mark_mem[next_cell][new_dir(next_dir, relU)] <= mark_mem[next_cell][new_dir(next_dir, relU)] + 2'd1;

        // --- Update Position and Counters ---
        current_cell <= next_cell;
        current_dir  <= next_dir;
        idx          <= idx + 1;
        
        // Update Coordinates for output
        x <= next_cell % 9;
        y <= next_cell / 9;

        brain_state <= WAIT_FOR_REQ; 
    end
end
endcase
end

end

reg [7:0] x, y;

endmodule

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
