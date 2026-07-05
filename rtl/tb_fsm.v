`timescale 1ns / 1ps

module tb_engine_fsm();
    reg clk;
    reg rst_n;
    reg [1:0] s0, s1, s2, s3, s4, s5, s6;
    
    wire [1:0] state;
    wire alarm, shutdown;

    engine_fsm uut (
        .clk(clk), .rst_n(rst_n),
        .s0_exh_temp(s0), .s1_lube_oil(s1), .s2_coolant(s2), .s3_start_air(s3),
        .s4_turbo_oil(s4), .s5_vibration(s5), .s6_plummer(s6),
        .engine_state(state), .alarm_led(alarm), .shutdown_led(shutdown)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst_n = 0;
        s0=0; s1=0; s2=0; s3=0; s4=0; s5=0; s6=0;
        #15 rst_n = 1;

        // T1: Normal operating conditions[cite: 1]
        #10;
        
        // T2: Exhaust Temp enters Warning (1 Warning -> MONITOR)[cite: 1]
        s0 = 2'b01; 
        #10;

        // T3: Vibration enters Warning (2 Warnings -> WARNING)[cite: 1]
        s5 = 2'b01; 
        #10;

        // T4: Critical Lube Oil Failure (Immediate Safety Interlock -> CRITICAL)[cite: 1]
        s1 = 2'b10; 
        #10;

        // T5: All clear and recovery[cite: 1]
        s0=0; s1=0; s5=0;
        #10;
        
        $finish;
    end
endmodule
