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
    wire  [19:0] encoder_counter_L_current, encoder_counter_R_current;     //reads encoder current value
    reg [19:0] L_ref, R_ref;
    wire [19:0] L_diff, R_diff, move_diff; 

    encoder enc_R(.clk_50M(clk_50M), .reset(reset), .en1(EN_A_R), .en2(EN_B_R), .counter(encoder_counter_R_current));
    encoder enc_L(.clk_50M(clk_50M), .reset(reset), .en1(EN_A_L), .en2(EN_B_L), .counter(encoder_counter_L_current));
    
    assign L_diff = (L_ref > encoder_counter_L_current) ? L_ref - encoder_counter_L_current : encoder_counter_L_current - L_ref;
    assign R_diff = (encoder_counter_R_current > R_ref) ? encoder_counter_R_current - R_ref : R_ref - encoder_counter_R_current;
    assign move_diff = (L_ref > encoder_counter_L_current) ? L_ref - encoder_counter_L_current : encoder_counter_L_current - L_ref; 
    
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
    // TUNING PARAMETERS
    // ===========================================================
    localparam DEAD_BAND = 16'd30;   //3cm
    localparam BASE_SPEED = 4'd10; 
    localparam TURN_ADJUST = 4'd2; 
    parameter RIGHT_MOTOR_TRIM = 1; // Add extra power to Right motor
    localparam front_US_turn_dim_up = 16'd100;  //10cm 

    // 3. GENERATORS & SENSORS
    frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));

    pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
    pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));
    
    // IR Sensor Logic
    ir i_front (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_F), .op(obst_f));
    ir i_left(.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_L), .op(obst_l));
    ir i_right (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_R), .op(obst_r));

    reg [15:0] diff;
    reg fa_turn_flag;       //flag to indicate to go to turn after forward after
    
    // ===========================================================
    // STATE MACHINE DEFINITIONS
    // ===========================================================
    localparam S_BOOT = 3'd0;  //boot time
    localparam S_FOLLOW  = 3'd1; // Normal Wall Following
    localparam S_TURN   = 3'd2; // Turning in place (90 degrees)
    localparam S_FORWARD_BEFORE = 3'd3;
    localparam S_FORWARD_AFTER = 3'd4;
    localparam S_STOP = 3'd5;
    localparam S_FILLER = 3'd6;
     
    (* keep *)  reg [2:0] state;           // Current State
    reg [2:0] prev_state;
    reg  [1:0] turn_left_mem;  // 1 = Left, 0 = Right, 2 = U_turn
    reg [31:0] state_timer;    // General purpose timer for states

    // TIMING CONSTANTS (Based on 50MHz Clock)
    localparam STOP_TIME_DELAY = 32'd25_000_000;  //0.5 seconds
    localparam BOOT_TIME_DELAY = 32'd100_000_000;  //2 second 
    
    localparam turn_R = 1310;
	 localparam turn_U = 1385;
    localparam turn_L = 1860; 
    localparam turn_FB = 2550;
    localparam turn_FA = 2900;
	 
(* keep *)	 reg [15:0] turn_count;
    
    
    always @(posedge clk_50M or negedge reset) begin
        if(!reset) begin
            state <= S_BOOT;
            state_timer <= BOOT_TIME_DELAY;
            turn_left_mem <= 0;
            fa_turn_flag <= 0;
				turn_count <=0;
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
                        if ((!obst_l && obst_r)) begin   // Left turn
                            turn_left_mem <= 2'd1;
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end else if ((obst_l && !obst_r)) begin  // Right turn
                            turn_left_mem <= 2'd0;
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end else if (!obst_l && !obst_r) begin     // Junction
									if((turn_count >=9 && turn_count < 12) || turn_count > 45) begin
									 turn_left_mem <= 2'd0;  // right priority 
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									end
									else begin
                            turn_left_mem <= 2'd1;  // Left priority 
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									 end
                        end else if (obst_f && obst_r && obst_l && (dF < front_US_turn_dim_up)) begin  // U-turn
                            turn_left_mem <= 2'd2;
                            state <= S_TURN;
                            prev_state <= S_FOLLOW;
                            state_timer <= STOP_TIME_DELAY;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									 turn_count <= turn_count + 1;
                        end
                    end
                end

                // ---------------------------------------------
                // FORWARD_BEFORE
                // ---------------------------------------------
                S_FORWARD_BEFORE: begin
                    if ((R_diff > turn_FB) || obst_f) begin  // how much should the bot go forward
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
                        if (prev_state == S_FORWARD_BEFORE || fa_turn_flag) begin
                            state <= S_TURN;
                            prev_state <= S_STOP;
                            state_timer <= 0;
                            fa_turn_flag <= 1'b0; // TO CLEAR THE FLAG
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									 turn_count <= turn_count + 1;
                        end
                        // Coming from TURN → go FORWARD_AFTER
                        else if (prev_state == S_TURN) begin
                            state <= S_FORWARD_AFTER;
                            prev_state <= S_STOP;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end
                    end else begin
                        state <= S_STOP;
                    end
                end  

                // ---------------------------------------------
                // TURN
                // ---------------------------------------------
                S_TURN: begin
                    if (turn_left_mem == 2'd1) begin // Left Turn
                        if (L_diff > turn_L) begin
                            state <= S_STOP;
                            state_timer <= STOP_TIME_DELAY;
                            prev_state <= S_TURN;
                        end else begin
                            state <= S_TURN;
                        end
                    end else if (turn_left_mem == 2'd0) begin // Right Turn
                        if (R_diff > turn_R) begin
                            state <= S_STOP;
                            state_timer <= STOP_TIME_DELAY;
                            prev_state <= S_TURN;
                        end else begin
                            state <= S_TURN;
                        end
                    end else begin // U-Turn
                        if (R_diff > ((2*turn_U) + 75)) begin
                            state <= S_STOP;
                            state_timer <= STOP_TIME_DELAY;
                            prev_state <= S_TURN;
                        end else begin
                            state <= S_TURN;
                        end
                    end
                end

                // ---------------------------------------------
                // FORWARD_AFTER
                // ---------------------------------------------
                S_FORWARD_AFTER: begin
                    if(obst_f) begin   // if obstacle in front, check for possible turns 
                        if(!obst_l && obst_r) begin    // Left turn
                            turn_left_mem <= 2'd1; 
                            state <= S_STOP;
                            prev_state <= S_FORWARD_AFTER; 
                            state_timer <= STOP_TIME_DELAY;
                            fa_turn_flag <= 1'b1;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end else if(obst_l && !obst_r) begin  // Right turn
                            turn_left_mem <= 2'd0;
                            state <= S_STOP;
                            prev_state <= S_FORWARD_AFTER;
                            state_timer <= STOP_TIME_DELAY;
                            fa_turn_flag <= 1'b1;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end 
								else if (!obst_l && !obst_r) begin     // Junction
									if((turn_count >=9 && turn_count < 12) || turn_count > 45) begin
									 turn_left_mem <= 2'd0;  // right priority 
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									end
									else begin
                            turn_left_mem <= 2'd1;  // Left priority 
                            state <= S_FORWARD_BEFORE;
                            prev_state <= S_FOLLOW;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
									 end
                        end
								else if(obst_l && obst_r) begin // U-turn
                            turn_left_mem <= 2'd2;
                            state <= S_STOP;
                            prev_state <= S_FORWARD_AFTER;
                            state_timer <= STOP_TIME_DELAY;
                            fa_turn_flag <= 1'b1;
                            L_ref <= encoder_counter_L_current;
                            R_ref <= encoder_counter_R_current;
                        end
                    end else if ((L_diff > turn_FA)) begin   // No obstacle, move forward done
                        state <= S_FOLLOW;
                        state_timer <= 0;
                        fa_turn_flag <= 1'b0;
                    end else begin
                        state <= S_FORWARD_AFTER;
                        fa_turn_flag <= 1'b0;
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

        // Calculate diff safely for unsigned logic
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
                // Simple Proportional Control Logic
                dt_cycle_right = (dR < 185) ? (15 - dR*15/AVERAGE_DISTANCE) : 8;
                dt_cycle_left  = (dL < 185) ? (15 - dL*15/AVERAGE_DISTANCE) : 8;
            end

            S_FORWARD_AFTER: begin
                IN1 = 1; IN2 = 0;
                IN3 = 1; IN4 = 0;
                dt_cycle_left  = BASE_SPEED;
                dt_cycle_right = BASE_SPEED;
            end

        endcase
    end
endmodule
