// engine_fsm.v
// Hardware-equivalent logic check for the PORT Main Engine
module engine_fsm (
    input wire clk,
    input wire rst_n,
    // 2-bit status codes for the 7 critical sensors (00=OK, 01=WARN, 10=ALARM)[cite: 1]
    input wire [1:0] s0_exh_temp,
    input wire [1:0] s1_lube_oil,
    input wire [1:0] s2_coolant,
    input wire [1:0] s3_start_air,
    input wire [1:0] s4_turbo_oil,
    input wire [1:0] s5_vibration,
    input wire [1:0] s6_plummer,
    
    output reg [1:0] engine_state, // 00=GOOD, 01=MONITOR, 10=WARNING, 11=CRITICAL[cite: 1]
    output reg alarm_led,
    output reg shutdown_led
);

    // Discrete State Parameters[cite: 1]
    localparam GOOD     = 2'b00;
    localparam MONITOR  = 2'b01;
    localparam WARNING  = 2'b10;
    localparam CRITICAL = 2'b11;

    reg [1:0] current_state, next_state;
    reg [2:0] warn_count;
    reg any_alarm;

    // Combinational evaluation of sensor inputs
    always @(*) begin
        // Count warnings
        warn_count = (s0_exh_temp == 2'b01) + (s1_lube_oil == 2'b01) + 
                     (s2_coolant  == 2'b01) + (s3_start_air == 2'b01) + 
                     (s4_turbo_oil == 2'b01) + (s5_vibration == 2'b01) + 
                     (s6_plummer  == 2'b01);
                     
        // Check for immediate emergency trip[cite: 1]
        any_alarm = (s0_exh_temp == 2'b10) || (s1_lube_oil == 2'b10) || 
                    (s2_coolant  == 2'b10) || (s3_start_air == 2'b10) || 
                    (s4_turbo_oil == 2'b10) || (s5_vibration == 2'b10) || 
                    (s6_plummer  == 2'b10);
    end

    // Sequential State Register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= GOOD;
        else
            current_state <= next_state;
    end

    // Next State Logic[cite: 1]
    always @(*) begin
        if (any_alarm) begin
            next_state = CRITICAL; // Direct safety trip bypass[cite: 1]
        end else begin
            case (current_state)
                GOOD:     next_state = (warn_count >= 2) ? WARNING : ((warn_count == 1) ? MONITOR : GOOD);
                MONITOR:  next_state = (warn_count >= 2) ? WARNING : ((warn_count == 0) ? GOOD : MONITOR);
                WARNING:  next_state = (warn_count == 1) ? MONITOR : ((warn_count == 0) ? GOOD : WARNING);
                CRITICAL: next_state = GOOD; // Recover to GOOD if cleared[cite: 1]
                default:  next_state = GOOD;
            endcase
        end
    end

    // Output Mapping[cite: 1]
    always @(*) begin
        engine_state = current_state;
        alarm_led    = (current_state == WARNING || current_state == CRITICAL);
        shutdown_led = (current_state == CRITICAL);
    end

endmodule
