(* DONT_TOUCH = "true"*)
module Test1(
    input clk_50M, 
    input reset,
    input echo1, echo2, echo3,

    output wire trig1, trig2, trig3,
    output op1, op2, op3, // Debug LEDs
    
    output wire ENA, ENB,       // PWM Enable (Speed)  ENA-right, ENB- left
    output reg IN1, IN2, IN3, IN4  // Motor Direction
);


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
	 
wire clk_3125KHz;
	 
frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));

wire [3:0] dt_cycle_left, dt_cycle_right;

pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));


assign dt_cycle_right = (dL < 200) ? (15 - dR*15/ AVERAGE_DISTANCE) : 6;
assign dt_cycle_left = (dR < 200) ? (15 - dL*15/ AVERAGE_DISTANCE) : 6;


always @(posedge clk_50M) begin
//Move forward
	IN1 <= 1;
	IN2 <= 0;
	IN3 <= 1;
	IN4 <= 0;
end

 

endmodule
