(* DONT_TOUCH = "true"*)
module Test1(
    input clk_50M, 
	 input ir_in_C, ir_in_R, ir_inL,
    input reset,
	 input EN_A_L,EN_A_R,
	 input EN_B_L,EN_B_R,
    input echo1, echo2, echo3,

    output wire trig1, trig2, trig3,
    output op1, op2, op3, // Debug LEDs
    
    output wire ENA, ENB,       // PWM Enable (Speed)  ENA-right, ENB- left
    output reg IN1, IN2, IN3, IN4  // Motor Direction
);
    wire [25:0] encoder_counter_R;
    wire [25:0] encoder_counter_L;

    // FIX 2: Fixed the missing parenthesis in enc_R
    encoder enc_L(.clk(clk_50M), .rst_n(reset), .A(EN_A_L), .B(EN_B_L), .counter(encoder_counter_L));
    encoder enc_R(.clk(clk_50M), .rst_n(reset), .A(EN_A_R), .B(EN_B_R), .counter(encoder_counter_R));
	

    // 2. INTERNAL WIRES
(* keep *)    wire [15:0] dL; // Left Distance
(* keep *)    wire [15:0] dF; // Front Distance
(* keep *)    wire [15:0] dR; // Right Distance



		//Ultrasonic Module
    Ultrasonic u0 (
        .clk_50M(clk_50M), .reset(reset), .echo1(echo1), .echo2(echo2), .echo3(echo3), .trig1(trig1), .trig2(trig2), .trig3(trig3),
		  .distance1(dF), 
        .distance2(dR), 
        .distance3(dL),
        .op1(op1), .op2(op2), .op3(op3)
    );
	 

parameter AVERAGE_DISTANCE = 200;
wire obst; 
wire clk_3125KHz;
	 
frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));

reg [3:0] dt_cycle_left, dt_cycle_right;

pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));
ir i_front (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_C),  .op(obst));

// Change these from wire to reg at the top of your module:
    // reg [3:0] dt_cycle_left, dt_cycle_right;

    // -----------------------------------------------------------
    //  TUNING PARAMETERS
    // -----------------------------------------------------------
    // Tolerance: 30mm (3cm). If error is less than this, go straight.
    localparam DEAD_BAND = 16'd30; 
    
    // Base Speed: Lower this if the bot is still too jerky.
    localparam BASE_SPEED = 4'd7; 
    
    // Correction: How much to turn? Keep this small (1, 2, or 3).
    localparam TURN_ADJUST = 4'd2; 

    reg [15:0] diff;

    always @(*) begin
        // 1. Calculate the difference between Left and Right
		IN1 = 1;
	IN2 = 0;
	IN3 = 1;
	IN4 = 0;
        if (dL > dR) diff = dL - dR;
        else         diff = dR - dL;

        // 2. LOGIC
		  if ( obst == 1'b1) begin 
		  IN1 = 1; IN2 = 0; IN3 = 0; IN4 = 1;
		  dt_cycle_right = 8;
		  dt_cycle_left = 5;
		  end 
		 
        else if (dL < 10 || dR < 10) begin
            // Safety: If sensors read garbage/zero, stop or cruise
            dt_cycle_right = BASE_SPEED;
            dt_cycle_left  = BASE_SPEED;
        end
        else if (diff < DEAD_BAND) begin
            // STABLE ZONE: We are roughly in the middle (+/- 3cm)
            // Just go straight! This fixes the "Wiggling".
            dt_cycle_right = BASE_SPEED;
            dt_cycle_left  = BASE_SPEED;
        end
        else if (dR < dL) begin
            // TOO RIGHT: Closer to right wall.
            // Action: Turn Left gently (Right motor faster)
            dt_cycle_right = BASE_SPEED + TURN_ADJUST;
            dt_cycle_left  = BASE_SPEED - TURN_ADJUST; 
        end
        else begin
            // TOO LEFT: Closer to left wall.
            // Action: Turn Right gently (Left motor faster)
            dt_cycle_right = BASE_SPEED - TURN_ADJUST;
            dt_cycle_left  = BASE_SPEED + TURN_ADJUST;
        end
    end

/*always @(posedge clk_50M) begin
//Move forward
	IN1 <= 0;
	IN2 <= 0;
	IN3 <= 0;
	IN4 <= 0;
end

 */

endmodule
