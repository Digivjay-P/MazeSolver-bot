(* DONT_TOUCH = "true"*)
module Test1(
    input clk_50M, 
    input ir_in_F, ir_in_R, ir_in_L, // Standardized Inputs
    input reset,
    input echo1, echo2, echo3,

    output wire trig1, trig2, trig3,
    output op1, op2, op3, // Debug LEDs
    
    output wire ENA, ENB,          // PWM Enable (Speed)
    output reg IN1, IN2, IN3, IN4  // Motor Direction
);

    // ==========================================
    // 1. MODULE INSTANTIATIONS (Sensors & PWM)
    // ==========================================
    
 
    
    // --- Ultrasonic Sensors ---
    (* keep *) wire [15:0] dL; 
    (* keep *) wire [15:0] dF; 
    (* keep *) wire [15:0] dR; 
    Ultrasonic u0 (
        .clk_50M(clk_50M), .reset(reset), .echo1(echo1), .echo2(echo2), .echo3(echo3), 
        .trig1(trig1), .trig2(trig2), .trig3(trig3),
        .distance1(dF), .distance2(dR), .distance3(dL),
        .op1(op1), .op2(op2), .op3(op3)
    );
    
    // --- PWM Generators ---
    wire clk_3125KHz;
    reg [3:0] dt_cycle_left, dt_cycle_right;
    frequency_scaling s1( .clk_50M(clk_50M), .clk_3125KHz(clk_3125KHz));
    pwm_generator right(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_right), .pwm_signal(ENA));
    pwm_generator left(.clk_3125KHz(clk_3125KHz), .duty_cycle(dt_cycle_left), .pwm_signal(ENB));

    // --- IR Sensors ---
    wire obst_f, obst_r, obst_l; 
    // Assuming ir module: 1 = Obstacle (Blocked), 0 = Clear
    ir i_front (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_F),  .op(obst_f));
    ir i_right (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_R),  .op(obst_r));
    ir i_left  (.clk_50M(clk_50M), .rst_n(reset), .ir_in(ir_in_L),  .op(obst_l));

    // ==========================================
    // 2. STATE MACHINE DEFINITIONS
    // ==========================================
    
    // Define states for readability
    localparam WALL_FOLLOW = 2'd0;
    localparam TURN_LEFT   = 2'd1;
    localparam TURN_RIGHT  = 2'd2;
    localparam U_TURN      = 2'd3;

    reg [1:0] state; // Current State

    // State Selection Logic
    always @(*) begin
        // PRIORITY: If Front is blocked, we MUST turn.
        if (obst_f == 1'b1) begin
            if (obst_l == 1'b0)      state = TURN_LEFT;   // Left is free? Take it.
            else if (obst_r == 1'b0) state = TURN_RIGHT;  // Right is free? Take it.
            else                     state = U_TURN;      // Dead End.
        end 
        else begin
            // Front is clear -> Follow the wall
            state = WALL_FOLLOW; 
        end
    end

    // ==========================================
    // 3. MOTION LOGIC (The "Brain")
    // ==========================================
    
    // Tuning Parameters
    localparam DEAD_BAND = 16'd30; 
    localparam BASE_SPEED = 4'd9;     
    localparam TURN_ADJUST = 4'd4;    
    reg [15:0] diff;

    always @(*) begin
        // Default initialization to prevent latches
        IN1 = 1; IN2 = 0; IN3 = 1; IN4 = 0; // Default Forward
        dt_cycle_right = BASE_SPEED;
        dt_cycle_left  = BASE_SPEED;
        
        // Calculate diff for Wall Follow logic
        if (dL > dR) diff = dL - dR;
        else         diff = dR - dL;

        case(state)
        
            // --- STATE: WALL FOLLOW (Go Straight + Correction) ---
            WALL_FOLLOW: begin
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

            // --- STATE: TURN LEFT (Pivot) ---
            TURN_LEFT: begin
                // "Use these turning speeds... Right=8, Left=5"
                // Logic: Pivot Left (Right FWD, Left BACK)
                IN1 = 1; IN2 = 0; // Right Fwd
                IN3 = 0; IN4 = 1; // Left Back
                
                dt_cycle_right = 4'd8; // Stronger
                dt_cycle_left  = 4'd5; // Weaker
            end

            // --- STATE: TURN RIGHT (Pivot) ---
            TURN_RIGHT: begin
                // Logic: Pivot Right (Right BACK, Left FWD)
                IN1 = 0; IN2 = 1; // Right Back
                IN3 = 1; IN4 = 0; // Left Fwd
                
                // Mirror the speeds from Left Turn
                dt_cycle_right = 4'd5; // Weaker
                dt_cycle_left  = 4'd8; // Stronger
            end

            // --- STATE: U-TURN (Dead End) ---
            U_TURN: begin
                // Spin hard to the Right
                IN1 = 0; IN2 = 1; 
                IN3 = 1; IN4 = 0; 
                
                dt_cycle_right = 4'd8;
                dt_cycle_left  = 4'd8;
            end
            
        endcase
    end

endmodule
