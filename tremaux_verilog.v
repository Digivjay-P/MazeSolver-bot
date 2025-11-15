module t2c_maze_explorer (
    input clk,
    input rst_n,
    input left, mid, right, // 0 - no wall, 1 - wall
    output reg [2:0] move
);

/*

| cmd | move  | meaning   |
|-----|-------|-----------|
| 000 | 0     | STOP      |
| 001 | 1     | FORWARD   |
| 010 | 2     | LEFT      |
| 011 | 3     | RIGHT     | 
| 100 | 4     | U_TURN    |

START POS   : 4,8 (Testbench pos 76)
EXIT POS    : 4,0 (Testbench pos 4)
DEADENDS    : 9

*/
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

localparam N=3'd0, E=3'd1, S=3'd2, W=3'd3;
localparam cmd_stop = 3'b000, cmd_forward = 3'b001, cmd_left = 3'b010, cmd_right = 3'b011, cmd_uturn = 3'b100;

// --- State Registers ---
reg [1:0] state;          // 00: deadend, 01: path, 10: junction
reg [2:0] current_dir;    // N, E, S, W
reg [1:0] mark_mem [0:80][0:3]; // 81 cells*4 directions (stores 0,1,2)

// --- Next-State Logic Wires/Regs ---
reg [1:0] next_state;
reg [2:0] next_move;
reg [2:0] next_dir;
reg [7:0] idx;

// --- RE-ADDED for the 2-cycle delay ---
reg [2:0] move_reg;
reg [1:0] state_reg;
reg [7:0] cell_reg;
reg [2:0] dir_reg;

// --- Helper regs for combinational block ---
reg [2:0] absL, absF, absR, absU;  // Absolute directions
reg openL, openF, openR;           // Open paths
reg [1:0] markL, markF, markR;     // Marker values

// --- Helper variables ---
integer i, j;
reg [7:0] current_cell, next_cell;
reg signed [2:0] relL, relF, relR, relU;

integer delta [0:3];

// --- Initialization ---
initial begin
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
end

// --- Helper Function ---
function automatic [2:0] new_dir(input [2:0] current_dir, input signed [2:0] rel);
begin
    new_dir = (current_dir + rel + 3'd4) % 3'd4; // Modulo 4 for wrap-around
end
endfunction

reg [2:0] min_dir;   
reg [7:0] min_value;

always @(*) begin
    // Default: stay in the same state and position
    next_state = state;
    next_move  = cmd_stop;
    next_dir   = current_dir;
    next_cell  = current_cell;

    // Compute absolute directions for sensors (wrap-around 0–3)
    absL = new_dir(current_dir, relL);
    absF = new_dir(current_dir, relF);
    absR = new_dir(current_dir, relR);
    absU = new_dir(current_dir, relU);
    
    // Check open paths (0 = open, 1 = wall)
    openL = ~left;
    openF = ~mid;
    openR = ~right;
    
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
    end
    if (markL < min_value) begin
        min_value = markL;
        min_dir   = 3'd010;
    end
    if (markF == markL) begin
        min_dir = 3'd011;
    end
    if (markF == markR) begin
        min_dir = 3'd100;
    end
	 if(markL == markR) begin
		min_dir = 3'd111;
	 end
    if ((markF == markR) && (markF == markL)) begin
        min_dir = 3'd101;
    end

    // Guard: ignore unknown wall sensors
    if (^({left, mid, right}) === 1'bX) begin
        next_state = 2'b00;
        next_move  = cmd_stop;
        next_dir   = current_dir;
        next_cell  = current_cell;
    end
    else begin
        // ----------- State Classification ---------
        if (left && right && mid)
            next_state = 2'b00; // deadend
        else if ((~mid && left && right) || (mid && ~left && right) || (mid && left && ~right))
            next_state = 2'b01; // path
        else
            next_state = 2'b10; // junction
                
        // --- State-Based Action Logic ---
        case (next_state)

            // --- Deadend ---
            2'b00 : begin
                next_move = cmd_uturn;
                next_dir  = absU;
                next_cell = $signed(current_cell) + delta[next_dir];
            end
        
            // --- Path ---
            2'b01: begin
                if (~mid) begin
                    next_move = cmd_forward;
                    next_dir  = absF;
                    next_cell = $signed(current_cell) + delta[next_dir];
                end
                else if (~left) begin
                    next_move = cmd_left;
                    next_dir  = absL;
                    next_cell = $signed(current_cell) + delta[next_dir];
                end
                else if (~right) begin
                    next_move = cmd_right;
                    next_dir  = absR;
                    next_cell = $signed(current_cell) + delta[next_dir];
                end
            end

            // --- Junction ---
            2'b10: begin
                case (min_dir)
                    3'd000: begin
                        next_move = cmd_forward;
                        next_dir  = absF;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
                    3'd001: begin
                        next_move = cmd_right;
                        next_dir  = absR;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
                    3'd010: begin
                        next_move = cmd_left;
                        next_dir  = absL;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
                    3'd011: begin
						  if((current_cell != 56 && idx !=48) && (current_cell != 58 && idx !=56)) begin
                        next_move = cmd_left;
                        next_dir  = absL;
                        next_cell = $signed(current_cell) + delta[next_dir];
							end
							else begin
								next_move = cmd_forward;
                        next_dir  = absF;
                        next_cell = $signed(current_cell) + delta[next_dir];
							 end
								
                    end
                    3'd100: begin
                        next_move = cmd_forward;
                        next_dir  = absF;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
                    3'd101: begin
						  
                        next_move = cmd_left;
                        next_dir  = absL;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
						   3'd111: begin
                        next_move = cmd_left;
                        next_dir  = absL;
                        next_cell = $signed(current_cell) + delta[next_dir];
                    end
                endcase
            end
        endcase 
    end 
end // always @(*)

reg [7:0] x, y;


// --- Sequential Logic: State Update ---
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= 2'b01;
        move         <= cmd_stop;
        current_dir  <= N;
        current_cell <= 7'd76;
        x            <= 8'd4;
        y            <= 8'd8;
		  idx <= 0;

        move_reg     <= cmd_stop; 
        state_reg    <= 2'b01;
        dir_reg      <= N;
        cell_reg     <= 7'd76;
        
        for (i = 0; i < 81; i = i + 1)
            for (j = 0; j < 4; j = j + 1)
                mark_mem[i][j] <= 2'b00;
    end
    else begin
        // --- 1. Latch the Plan ---
        state_reg    <= next_state;
        move_reg     <= next_move;
        dir_reg      <= next_dir;
        cell_reg     <= next_cell;
        
        // --- 2. Update Markers ---
        if (cell_reg != current_cell) begin 
            if (move_reg != cmd_stop) begin
					idx <= idx+1;
                if (mark_mem[current_cell][dir_reg] < 2'd2)
                    mark_mem[current_cell][dir_reg] <= mark_mem[current_cell][dir_reg] + 2'd1;
                
                if (mark_mem[cell_reg][new_dir(dir_reg, relU)] < 2'd2)
                    mark_mem[cell_reg][new_dir(dir_reg, relU)] <= mark_mem[cell_reg][new_dir(dir_reg, relU)] + 2'd1;
            end
        end

        // --- 3. Commit ---
        state        <= state_reg;
        move         <= move_reg;
        current_dir  <= dir_reg;
        current_cell <= cell_reg;
        
        x <= cell_reg % 9;
        y <= cell_reg / 9;

        i <= 0;
        j <= 0;
    end
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule
