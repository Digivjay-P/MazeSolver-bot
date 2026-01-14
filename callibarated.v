(* DONT_TOUCH = "true"*)
module Test1(
    input clk_50M, 
    input ir_in_F, ir_in_R, ir_in_L,
    input reset,        // Assumed Active Low based on enc_L usage
    input EN_A_L, EN_A_R,
    input EN_B_L, EN_B_R,
    input echo1, echo2, echo3,
    output wire trig1, trig2, trig3,
    output op1, op2, op3, // Debug LEDs
    output wire ENA, ENB,       // PWM Enable (Speed)
    output reg IN1, IN2, IN3, IN4  // Motor Direction
);
    
    // 2. INTERNAL WIRES
    (* keep *) wire [15:0] dL; // Left Distance
    (* keep *) wire [15:0] dF; // Front Distance
    (* keep *) wire [15:0] dR; // Right Distance
    wire  [19:0] encoder_counter_L_current, encoder_counter_R_current;	 //reads encoder current value
	 reg [19:0] L_ref, R_ref;
	 wire [19:0] L_diff, R_diff, move_diff; 
	 encoder enc_R(.clk_50M(clk_50M), .reset(reset), .en1(EN_A_R), .en2(EN_B_R), .counter(encoder_counter_R_current));
    encoder enc_L(.clk_50M(clk_50M), .reset(reset), .en1(EN_A_L), .en2(EN_B_L), .counter(encoder_counter_L_current));
	 assign L_diff = (L_ref > encoder_counter_L_current) ? L_ref - encoder_counter_L_current : encoder_counter_L_current - L_ref;
	 assign R_diff = (encoder_counter_R_current > R_ref) ? encoder_counter_R_current - R_ref : R_ref - encoder_counter_R_current;
	 assign move_diff = (L_ref > encoder_counter_L_current) ? L_ref - encoder_counter_L_current : encoder_counter_L_current - L_ref; //move_diff using left should be positive when bot moves straight

    
    // Ultrasonic Module
    Ultrasonic u0 (
        .clk_50M(clk_50M), .reset(reset), .echo1(echo1), .echo2(echo2), .echo3(echo3), 
        .trig1(trig1), .trig2(trig2), .trig3(trig3),
        .distance1(dF), 
        .distance2(dR), 
        .distance3(dL),
        .op1(op1), .op2(op2), .op3(op3)
    );
    wire obst_f, obst_r, obst_l; 
    wire clk_3125KHz;
	 (* keep *)    reg [3:0] dt_cycle_left, dt_cycle_right;

	 
	 
    localparam AVERAGE_DISTANCE = 200;
	 
	     localparam LOCKOUT_TIME = 32'd80_000_000; // 1.5 Seconds

		  
     // ===========================================================
    // -----------------------------------------------------------
    //  TUNING PARAMETERS
    // -----------------------------------------------------------
    localparam DEAD_BAND = 16'd30;   //3cm
    localparam BASE_SPEED = 4'd10; 
    localparam TURN_ADJUST = 4'd2; 
	 parameter RIGHT_MOTOR_TRIM = 1; // Add extra power to Right motor
	 localparam front_US_turn_dim_up = 16'd100;  //10cm 
    // 3. GENERATORS & SENSORS
    frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));

    // Changed to reg as required for procedural assignment

    pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
    pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));
    
    // Front IR Sensor Logic
    // obst becomes 1 when Wall is detected
    ir i_front (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_F), .op(obst_f));
	 ir i_left(.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_L), .op(obst_l));
	 ir i_right (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_R), .op(obst_r));


    

    reg [15:0] diff;
	
//  NEW: STATE MACHINE DEFINITIONS
    // ===========================================================
	 localparam S_BOOT = 3'd0;  //boot time
    localparam S_FOLLOW  = 3'd1; // Normal Wall Following
    localparam S_TURN   = 3'd2; // Turning in place (90 degrees)
    localparam S_FORWARD_BEFORE = 3'd3;
	 localparam S_FORWARD_AFTER = 3'd4;
	 localparam S_STOP = 3'd5;
	 localparam S_FILLER = 3'd6;
	 
(* keep *)	 reg [2:0] state;          // Current State
reg [2:0] prev_state;
				 reg  [1:0] turn_left_mem;  // 1 = Left, 0 = Right 2= U_turn; (Remembers direction)
				 reg [31:0] state_timer;   // General purpose timer for states

    // TIMING CONSTANTS (Based on 50MHz Clock)
    // 0.5 Seconds = 25,000,000 cycles
    // Adjust TURN_TIME_DELAY to get exactly 90 degrees rotation
	 localparam STOP_TIME_DELAY = 32'd25_000_000;  //0.5 seconds
	 localparam BOOT_TIME_DELAY = 32'd100_000_000;  //2 second 
    //localparam FORWARD_TIME_DELAY_AFTER= 32'd37_500_000; // 0.75 (Fixed delay)
	// localparam FORWARD_TIME_DELAY_BEFORE= 32'd25_000_000; // .5s (Fixed delay)
	localparam turn_R= 1372;
	localparam turn_L = 1874;  //earlier 1700
	localparam turn_FB = 2550;
	localparam turn_FA = 2900;
always @(posedge clk_50M or negedge reset) begin
    if(!reset) begin
        state <= S_BOOT;
        state_timer <= BOOT_TIME_DELAY;
        turn_left_mem <= 0;
    end else begin

        // Count down timer
        if (state_timer > 0)
            state_timer <= state_timer - 1;

        case(state)

        // ---------------------------------------------
        // BOOT
        // ---------------------------------------------
        S_BOOT: begin
            if (state_timer == 0) begin
                state <= S_FOLLOW;
                prev_state <= S_BOOT;
            end else begin
                state <= S_BOOT;
            end
        end

        // ---------------------------------------------
        // FOLLOW
        // ---------------------------------------------
        S_FOLLOW: begin
            state_timer <= 0;

            if(!obst_f && obst_r && obst_l) begin
                state <= S_FOLLOW;
            end else begin

                if ((!obst_l && obst_r) && dL > 250) begin   //left turn
                    turn_left_mem <= 2'd1;
                    state <= S_FORWARD_BEFORE;
                    prev_state <= S_FOLLOW;
                    L_ref <= encoder_counter_L_current;
                    R_ref <= encoder_counter_R_current;

                end else if ((obst_l && !obst_r) && dR > 250) begin  //right turn
                    turn_left_mem <= 2'd0;
                    state <= S_FORWARD_BEFORE;
                    prev_state <= S_FOLLOW;
                    L_ref <= encoder_counter_L_current;
                    R_ref <= encoder_counter_R_current;

                end else if (!obst_l && !obst_r) begin  //junction
                    turn_left_mem <= 2'd1;  //left priority
                    state <= S_FORWARD_BEFORE;
                    prev_state <= S_FOLLOW;
                    L_ref <= encoder_counter_L_current;
                    R_ref <= encoder_counter_R_current;

                end else if (obst_f && obst_r && obst_l && (dF < front_US_turn_dim_up)) begin  // uturn
                    turn_left_mem <= 2'd2;
                    state <= S_TURN;
                    prev_state <= S_FOLLOW;
                    state_timer <= STOP_TIME_DELAY;
						  L_ref <= encoder_counter_L_current;
						  R_ref <= encoder_counter_R_current;
                end

            end
        end

        // ---------------------------------------------
        // FORWARD_BEFORE
        // ---------------------------------------------
        S_FORWARD_BEFORE: begin
            if ((R_diff > turn_FB) ) begin  //how much should the bot go forward
                state <= S_STOP;
                prev_state <= S_FORWARD_BEFORE;
                state_timer <= STOP_TIME_DELAY;
            end else begin
                state <= S_FORWARD_BEFORE;
            end
        end

// ---------------------------------------------
// STOP
// ---------------------------------------------
S_STOP: begin
    if(state_timer == 0) begin

        // Coming from FORWARD_BEFORE → go TURN
        if (prev_state == S_FORWARD_BEFORE) begin
            state <= S_TURN;
            prev_state <= S_STOP;
            state_timer <= 0;
            L_ref <= encoder_counter_L_current;
            R_ref <= encoder_counter_R_current;
        end

        // Coming from TURN → go FORWARD_AFTER
        else if (prev_state == S_TURN) begin
            state <= S_FORWARD_AFTER;
            prev_state <= S_STOP;
            state_timer <= 32'd15_000_000;
        end

        // Coming from FORWARD_AFTER
        else if (prev_state == S_FORWARD_AFTER) begin

            // If front blocked → turn again
            if (obst_f) begin
                state <= S_TURN;
                prev_state <= S_STOP;
                state_timer <= 0;
                L_ref <= encoder_counter_L_current;
                R_ref <= encoder_counter_R_current;
            end

            // If left AND right walls → FOLLOW
            else if (obst_l && obst_r) begin
                state <= S_FOLLOW;
                prev_state <= S_STOP;
                state_timer <= 0;
                L_ref <= encoder_counter_L_current;
                R_ref <= encoder_counter_R_current;
            end

            // Else → continue forward (FILLER)
            else begin
                state <= S_FILLER;
                prev_state <= S_STOP;
                state_timer <= 0;
                L_ref <= encoder_counter_L_current;
                R_ref <= encoder_counter_R_current;
            end
        end

        // Fallback stay
        else begin
            state <= S_STOP;
        end

    end else begin
        state <= S_STOP;
    end
end

       
        // ---------------------------------------------
        // TURN
        // ---------------------------------------------
        S_TURN: begin

            if (turn_left_mem == 2'd1) begin
                if (L_diff > turn_L) begin
                    state <= S_STOP;
                    state_timer <= STOP_TIME_DELAY;
                    prev_state <= S_TURN;
                end else state <= S_TURN;

            end else if (turn_left_mem == 2'd0) begin
                if (R_diff > turn_R) begin
                    state <= S_STOP;
                    state_timer <= STOP_TIME_DELAY;
                    prev_state <= S_TURN;
                end else state <= S_TURN;

            end else begin // U-Turn
				if (R_diff > ((2*turn_R) + 40)) begin
                    state <= S_STOP;
                    state_timer <= STOP_TIME_DELAY;
                    prev_state <= S_TURN;
                end else state <= S_TURN;
            end
        end

        // ---------------------------------------------
        // FILLER
        // ---------------------------------------------
      S_FILLER: begin
    
    // If front blocked → choose turn direction
    if (dF < 140) begin
        
        // Check for actual side walls first
        if (obst_l && !obst_r) begin
            // Left wall present, right side open → turn RIGHT
            turn_left_mem <= 2'd0;
        end
        else if (!obst_l && obst_r) begin
            // Right wall present, left open → turn LEFT
            turn_left_mem <= 2'd1;
        end
        else if (!obst_l && !obst_r) begin
            // Both sides open → default to left turn
            turn_left_mem <= 2'd1;
        end
        else begin
            // Rare case: both walls detected + front blocked → U-turn
            turn_left_mem <= 2'd2;
        end

        // perform the turn
        state <= S_TURN;
        prev_state <= S_FILLER;
        L_ref <= encoder_counter_L_current;
        R_ref <= encoder_counter_R_current;
    end
    
    // If both walls detected → resume FOLLOW
    else if (obst_l && obst_r) begin
        state <= S_FOLLOW;
        prev_state <= S_FILLER;
        L_ref <= encoder_counter_L_current;
        R_ref <= encoder_counter_R_current;
    end
    
    // No obstacle, no walls → keep moving straight
    else begin
        state <= S_FILLER;
    end

end


        // ---------------------------------------------
        // FORWARD_AFTER
        // ---------------------------------------------
        S_FORWARD_AFTER: begin
            if ((L_diff > turn_FA) ) begin
                state <= S_STOP;
                prev_state <= S_FORWARD_AFTER;
                state_timer <= STOP_TIME_DELAY;
            end else begin
                state <= S_FORWARD_AFTER;
            end
        end

        endcase
    end
end


// =====================================================
// MOTOR LOGIC ALWAYS BLOCK
// =====================================================
always @(*) begin
    
    // Default forward
    IN1 = 1; IN2 = 0;
    IN3 = 1; IN4 = 0;

    if (dL > dR && dR > 0)
        diff = dL - dR;
    else
        diff = dR - dL;

    case(state)

    S_BOOT: begin
        IN1 = 1; IN2 = 1;
        IN3 = 1; IN4 = 1;
        dt_cycle_left = 0;
        dt_cycle_right = 0;
    end

    S_STOP: begin
        IN1 = 1; IN2 = 1;
        IN3 = 1; IN4 = 1;
        dt_cycle_left = 0;
        dt_cycle_right = 0;
    end

    S_TURN: begin
        if(turn_left_mem == 2'd1) begin
            IN1 = 1; IN2 = 0;
            IN3 = 0; IN4 = 1;
        end else if(turn_left_mem == 2'd0) begin
            IN1 = 0; IN2 = 1;
            IN3 = 1; IN4 = 0;
        end else begin
            IN1 = 1; IN2 = 0;
            IN3 = 0; IN4 = 1;
        end

        dt_cycle_left = 7;
        dt_cycle_right = 7;
    end

    S_FORWARD_BEFORE: begin
        IN1 = 1; IN2 = 0;
        IN3 = 1; IN4 = 0;
        dt_cycle_left = BASE_SPEED;
        dt_cycle_right = BASE_SPEED;
    end

    S_FOLLOW: begin
        IN1 = 1; IN2 = 0;
        IN3 = 1; IN4 = 0;

        dt_cycle_right = (dR < 185) ? (15 - dR*15/AVERAGE_DISTANCE) : 8;
        dt_cycle_left  = (dL < 185) ? (15 - dL*15/AVERAGE_DISTANCE) : 8;
    end

    S_FORWARD_AFTER: begin
        IN1 = 1; IN2 = 0;
        IN3 = 1; IN4 = 0;
        dt_cycle_left  = BASE_SPEED;
        dt_cycle_right = BASE_SPEED;
    end

    S_FILLER: begin
        IN1 = 1; IN2 = 0;
        IN3 = 1; IN4 = 0;
        dt_cycle_left  = BASE_SPEED;
        dt_cycle_right = BASE_SPEED;
    end

    endcase
end
endmodule 
