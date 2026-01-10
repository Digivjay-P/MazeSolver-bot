(* DONT_TOUCH = "true"*)
module Test1(
    input clk_50M, 
    input ir_in_C, ir_in_R, ir_inL,
    input reset,        // Assumed Active Low based on enc_L usage
    input EN_A_L, EN_A_R,
    input EN_B_L, EN_B_R,
    input echo1, echo2, echo3,
	 output reg LED, LEDS1, LEDS2, LEDS3,
    output wire trig1, trig2, trig3,
    output op1, op2, op3, // Debug LEDs
    
    output wire ENA, ENB,       // PWM Enable (Speed)
    output reg IN1, IN2, IN3, IN4  // Motor Direction
);

    // 1. WIRES & ENCODERS
    wire [25:0] encoder_counter_R;
    wire [25:0] encoder_counter_L;

    encoder enc_L(.clk(clk_50M), .rst_n(reset), .A(EN_A_L), .B(EN_B_L), .counter(encoder_counter_L));
    encoder enc_R(.clk(clk_50M), .rst_n(reset), .A(EN_A_R), .B(EN_B_R), .counter(encoder_counter_R));
    
    // 2. INTERNAL WIRES
    (* keep *) wire [15:0] dL; // Left Distance
    (* keep *) wire [15:0] dF; // Front Distance
    (* keep *) wire [15:0] dR; // Right Distance
    
    // Ultrasonic Module
    Ultrasonic u0 (
        .clk_50M(clk_50M), .reset(reset), .echo1(echo1), .echo2(echo2), .echo3(echo3), 
        .trig1(trig1), .trig2(trig2), .trig3(trig3),
        .distance1(dF), 
        .distance2(dR), 
        .distance3(dL),
        .op1(op1), .op2(op2), .op3(op3)
    );
    
    parameter AVERAGE_DISTANCE = 200;
    wire obst; 
    wire clk_3125KHz;
    
    // 3. GENERATORS & SENSORS
    frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));

    // Changed to reg as required for procedural assignment
(* keep *)    reg [3:0] dt_cycle_left, dt_cycle_right;

    pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
    pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));
    
    // Front IR Sensor Logic
    // obst becomes 1 when Wall is detected
    ir i_front (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_C), .op(obst));



    // ===========================================================
    //  ROBUST TURN COUNTER (With Cooldown/Debounce)
    // ===========================================================
    reg [7:0] turn_count;      // Stores the number of turns
    reg obst_prev;             // To store previous state
    reg [31:0] cooldown_timer; // Timer to ignore noise after a turn

    // CONFIGURATION: Cooldown Time
    // 50,000,000 cycles = 1 second. 
    // Adjust this based on how long your robot takes to turn.
    // If it double counts, INCREASE this. If it misses the next wall, DECREASE this.
    localparam LOCKOUT_TIME = 32'd75_000_000; // 1.5 Seconds

    always @(posedge clk_50M or negedge reset) begin
        if (reset == 0) begin
            turn_count <= 0;
            obst_prev <= 0;
            cooldown_timer <= 0;
        end else begin
            
            // 1. If the timer is running, just count down and IGNORE sensors
            if (cooldown_timer > 0) begin
                cooldown_timer <= cooldown_timer - 1;
                // We do NOT update obst_prev here to prevent edge glitches
            end 
            
            // 2. If timer is 0, we are ready to detect a new turn
            else begin
                // Detect Rising Edge (0 -> 1)
                if (obst == 1'b1 && obst_prev == 1'b0) begin
                    turn_count <= turn_count + 1;
                    
                    // START THE COOLDOWN TIMER
                    // The counter will now be "frozen" for 1.5 seconds
                    cooldown_timer <= LOCKOUT_TIME; 
                end
                
                // Update previous state
                obst_prev <= obst; 
            end
        end
    end
    // ===========================================================
    // -----------------------------------------------------------
    //  TUNING PARAMETERS
    // -----------------------------------------------------------
    localparam DEAD_BAND = 16'd30;   //3cm
    localparam BASE_SPEED = 4'd10; 
    localparam TURN_ADJUST = 4'd5; 
	 
	 //when front US shows bw 11 and 13 cm checking turn 
	 localparam front_US_turn_dim_up = 16'd200; 
	  // localparam front_US_turn_dim_down = 16'd110; 

    reg [15:0] diff;

//  NEW: STATE MACHINE DEFINITIONS
    // ===========================================================
    localparam S_FOLLOW  = 2'd0; // Normal Wall Following
    localparam S_TURN   = 2'd1; // Turning in place (90 degrees)
    localparam S_FORWARD_BEFORE= 2'd2; // Blind forward move after turn
	 localparam S_FORWARD_AFTER= 2'd3;
(* keep *)	 reg [1:0] state;          // Current State
	 reg       turn_left_mem;  // 1 = Left, 0 = Right (Remembers direction)
    reg [31:0] state_timer;   // General purpose timer for states

    // TIMING CONSTANTS (Based on 50MHz Clock)
    // 0.5 Seconds = 25,000,000 cycles
    // Adjust TURN_TIME_DELAY to get exactly 90 degrees rotation
    localparam TURN_TIME_DELAY    = 32'd37_500_000; // .5s (Tune this!)
    //localparam FORWARD_TIME_DELAY_AFTER= 32'd37_500_000; // 0.75 (Fixed delay)
	// localparam FORWARD_TIME_DELAY_BEFORE= 32'd25_000_000; // .5s (Fixed delay)

	 
	 
	 
	 always @(posedge clk_50M or negedge reset) begin
		if(!reset) begin
			state <= S_FOLLOW;
			state_timer <= 0;
            turn_left_mem <= 0;
		end
		else begin
			// --- STATE MACHINE TRANSITIONS ---
            if (state_timer > 0) begin
                state_timer <= state_timer - 1;
            end
				else begin
					//Timer expired begin to make changes
					case(state)
						S_FOLLOW : begin
							if(dF < front_US_turn_dim_up) begin
								state <= S_FORWARD_BEFORE;
								state_timer <= 00 ;
								if (dL > dR) 
                                turn_left_mem <= 1'b1; // Turn Left
                            else 
                                turn_left_mem <= 1'b0; // Turn Right
						       end
				         end
						S_FORWARD_BEFORE :begin
                                if(dF<150) begin
								state <= S_TURN;
								state_timer <= TURN_TIME_DELAY;
                                end
                                else 
                                state <= S_FORWARD_BEFORE;
						
						end
						S_TURN : begin
							// Turn is done, now force move forward
							state <= S_FORWARD_AFTER;
							state_timer <= 0;   //Delay of 0.5 second //loading the timer
						end
						
						S_FORWARD_AFTER: begin
							// Forward burst done, resume wall following
                            if(dF> 310) 
							state <=S_FOLLOW;
                            else if (dF<= 150) begin
									 state <= S_FOLLOW;
									 end
							
						end
				endcase
		  end
	 end
     end
    // MOTOR LOGIC
    always @(*) begin
        // Default Motor Direction (Forward)
        IN1 = 1; IN2 = 0; 
        IN3 = 1; IN4 = 0;

        // Calculate Difference
        if (dL > dR && dR > 0) begin 
		  diff = dL - dR;
		  end
        else         diff = dR - dL;
		  
	case(state)
		S_TURN : begin
			if(turn_left_mem == 1'b1) begin           //take a left turn on the spot
				IN1 = 1; IN2 = 0; 
            IN3 = 0; IN4 = 1;
				// Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
			end
			else begin              //take a right turn on the spot 
				IN1 = 0; IN2 = 1; 
            IN3 = 1; IN4 = 0; 
            
            // Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
		end
	  end
		S_FORWARD_BEFORE: begin   //go straight
			IN1 = 1; IN2 = 0; // Right Forward
         IN3 = 1; IN4 = 0; // Left Forward
			
			dt_cycle_right = BASE_SPEED;
         dt_cycle_left = BASE_SPEED;
		end
		S_FORWARD_AFTER: begin   //go straight
			IN1 = 1; IN2 = 0; // Right Forward
         IN3 = 1; IN4 = 0; // Left Forward
			
			dt_cycle_right = BASE_SPEED;
         dt_cycle_left = BASE_SPEED;
		end
		S_FOLLOW : begin
				  IN1 = 1; IN2 = 0; // Right Forward
              IN3 = 1; IN4 = 0; // Left Forward
                
              if (dL < 10 || dR < 10 || diff < DEAD_BAND) begin
                    // Go Straight (Stable or Safety)
                    dt_cycle_right = BASE_SPEED;
                    dt_cycle_left  = BASE_SPEED;
              end
              else if (dR < dL) begin
                    // Too Close to Right -> Turn Left Gently
                    dt_cycle_right = BASE_SPEED + TURN_ADJUST;
                    dt_cycle_left  = BASE_SPEED - TURN_ADJUST; 
              end
              else begin
                    // Too Close to Left -> Turn Right Gently
                    dt_cycle_right = BASE_SPEED - TURN_ADJUST;
                    dt_cycle_left  = BASE_SPEED + TURN_ADJUST;
              end
			end
	endcase
	if( turn_count == 4) begin 
	LED <= 1'b1;
	end 
	if (state == 0) 
	LEDS1 <= 1;
	
   if ( state == 1)
	LEDS2 <= 1;
   if (state == 2)
	LEDS3 <= 1;
end
		  
      /* if( dF < front_US_turn_dim_up) begin
			if( dL > dR )begin   //left distance more than right thus turn left
				IN1 = 1; IN2 = 0; 
            IN3 = 0; IN4 = 1;
				// Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
			end
			else begin //turn right (spot turn)
            IN1 = 0; IN2 = 1; 
            IN3 = 1; IN4 = 0; 
            
            // Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
			end
		 end
		 
		 else begin  // Keep Wall Following 

				  IN1 = 1; IN2 = 0; // Right Forward
              IN3 = 1; IN4 = 0; // Left Forward
                
              if (dL < 10 || dR < 10 || diff < DEAD_BAND) begin
                    // Go Straight (Stable or Safety)
                    dt_cycle_right = BASE_SPEED;
                    dt_cycle_left  = BASE_SPEED;
              end
              else if (dR < dL) begin
                    // Too Close to Right -> Turn Left Gently
                    dt_cycle_right = BASE_SPEED + TURN_ADJUST;
                    dt_cycle_left  = BASE_SPEED - TURN_ADJUST; 
              end
              else begin
                    // Too Close to Left -> Turn Right Gently
                    dt_cycle_right = BASE_SPEED - TURN_ADJUST;
                    dt_cycle_left  = BASE_SPEED + TURN_ADJUST;
              end
           end	
			
						 	 
			if( turn_count == 4) LED <= 1'b1;
		 end */
		 
		  
		/*
        // ------------------------------------
        // PRIORITY 1: OBSTACLE (Turn Logic)
        // ------------------------------------
       if ( obst) begin 
		 if ( turn_count == 2 || turn_count == 5) begin //Right
            // Turn Right (Spot Turn)
            IN1 = 1; IN2 = 0; 
            IN3 = 0; IN4 = 1; 
            
            // Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
        end 
		  else if ( turn_count == 2 || turn_count == 3 || turn_count ==  ) begin //Left
		    IN1 = 0; IN2 = 1; 
            IN3 = 1; IN4 = 0; 
           
            // Speed for turning
            dt_cycle_right = 7;
            dt_cycle_left = 7;
		  end 
		  else if ( turn_count == 6 ) begin 
		   IN1 = 0; IN2 = 1; 
            IN3 = 0; IN4 = 1; 
				 dt_cycle_right = 11;
            dt_cycle_left = 8;
				end 
        end 
        // ------------------------------------
        // PRIORITY 2: SENSOR SAFETY
        // ------------------------------------
        else if (dL < 10 || dR < 10) begin
            dt_cycle_right = BASE_SPEED;
            dt_cycle_left  = BASE_SPEED;
        end
        
        // ------------------------------------
        // PRIORITY 3: WALL FOLLOWING
        // ------------------------------------
        else if (diff < DEAD_BAND) begin
            // Go Straight
            dt_cycle_right = BASE_SPEED;
            dt_cycle_left  = BASE_SPEED;
        end
        else if (dR < dL) begin
            // Too Close to Right -> Turn Left
            dt_cycle_right = BASE_SPEED + TURN_ADJUST;
            dt_cycle_left  = BASE_SPEED - TURN_ADJUST; 
        end
        else begin
            // Too Close to Left -> Turn Right
            dt_cycle_right = BASE_SPEED - TURN_ADJUST;
            dt_cycle_left  = BASE_SPEED + TURN_ADJUST;
        end
		  */

endmodule
