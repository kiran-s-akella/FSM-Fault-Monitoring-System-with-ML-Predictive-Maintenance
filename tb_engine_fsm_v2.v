`timescale 1ms/1us

// ════════════════════════════════════════════════════════════════════════
// tb_engine_fsm_v2.v
// Testbench for engine_fsm_v2 — exercises all 7 sensors with scenarios
// that mirror the synthetic data faults from the Python ML pipeline:
//
//   T1: Normal operation — all 7 sensors in healthy band
//   T2: Exhaust temp injector fouling  -> WARN -> FAULT (matches 580/600°C)
//   T3: Lube oil pressure dip          -> WARN (matches day 55-57 dip)
//   T4: Vibration imbalance fault      -> WARN -> FAULT (matches day 50+)
//   T5: Plummer bearing oil restriction -> WARN -> FAULT (matches day 60+)
//   T6: Multiple simultaneous faults   -> common_shutdown asserted
//   T7: Recovery — all sensors back to normal, prelube ok, ack
// ════════════════════════════════════════════════════════════════════════

module tb_engine_fsm_v2;

    reg clk, rst_n;

    // ADC inputs for each sensor
    reg [11:0] exh_temp_adc;
    reg [11:0] lube_oil_adc;
    reg [11:0] coolant_adc;
    reg [11:0] start_air_adc;
    reg [11:0] turbo_oil_adc;
    reg [11:0] vibration_adc;
    reg [11:0] plummer_temp_adc;

    // Common signals
    reg emerg_stop, overspeed, prelube_ok, alarm_ack, fault_cleared;
    reg fmi_open_ckt, fmi_short;

    // Outputs
    wire do_common_alarm, do_common_shutdown, do_engine_ready;
    wire [2:0] state_exh_temp, state_lube_oil, state_coolant;
    wire [2:0] state_start_air, state_turbo_oil, state_vibration, state_plummer;
    wire [6:0] sensor_fail_mask;
    wire [6:0] engine_health_pct;

    engine_fsm_v2 dut (
        .clk(clk), .rst_n(rst_n),
        .exh_temp_adc(exh_temp_adc),
        .lube_oil_adc(lube_oil_adc),
        .coolant_adc(coolant_adc),
        .start_air_adc(start_air_adc),
        .turbo_oil_adc(turbo_oil_adc),
        .vibration_adc(vibration_adc),
        .plummer_temp_adc(plummer_temp_adc),
        .emerg_stop(emerg_stop), .overspeed(overspeed),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack),
        .fault_cleared(fault_cleared),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .do_common_alarm(do_common_alarm),
        .do_common_shutdown(do_common_shutdown),
        .do_engine_ready(do_engine_ready),
        .state_exh_temp(state_exh_temp),
        .state_lube_oil(state_lube_oil),
        .state_coolant(state_coolant),
        .state_start_air(state_start_air),
        .state_turbo_oil(state_turbo_oil),
        .state_vibration(state_vibration),
        .state_plummer(state_plummer),
        .sensor_fail_mask(sensor_fail_mask),
        .engine_health_pct(engine_health_pct)
    );

    // 100 Hz clock — 10ms period
    initial clk = 0;
    always #5 clk = ~clk;

    // ── NORMAL operating values (centre of normal_op band, converted to ADC) ──
    // EXH_PORT_TEMP (RTD, 0-4095 mapping): 480°C -> (480-200)/500*4095 = 2293
    localparam EXH_NORMAL  = 12'd2293;
    // LUBE_OIL_PRES (4-20mA, 800-4095 mapping): 310 kPa -> 2843
    localparam OIL_NORMAL  = 12'd2843;
    // COOLANT_PRES (4-20mA): 90 kPa -> 2777
    localparam COOL_NORMAL = 12'd2777;
    // START_AIR_PRES (4-20mA): 1200 kPa -> 3126
    localparam AIR_NORMAL  = 12'd3126;
    // TURBO_OIL_PRES (4-20mA): 220 kPa -> 1404
    localparam TURB_NORMAL = 12'd1404;
    // VIBRATION_RMS (4-20mA): 3.5 mm/s -> 1261
    localparam VIB_NORMAL  = 12'd1261;
    // PLUMMER_BRG_TEMP (RTD, 0-4095 mapping): 50°C -> 1228
    localparam PLUM_NORMAL = 12'd1228;

    // ── FAULT values (computed from Python WARN/ALARM thresholds) ──
    localparam EXH_WARN    = 12'd3150;  // > 3112 -> 580°C+
    localparam EXH_FAULT   = 12'd3300;  // > 3276 -> 600°C+
    localparam OIL_WARN    = 12'd1600;  // < 1788 -> <150kPa  (still >800, valid signal)
    localparam OIL_FAULT   = 12'd1300;  // < 1459 -> <100kPa
    localparam VIB_WARN    = 12'd2100;  // > 1986 -> >9.0mm/s
    localparam VIB_FAULT   = 12'd2900;  // > 2645 -> >14.0mm/s
    localparam PLUM_WARN   = 12'd2300;  // > 2252 -> >75°C
    localparam PLUM_FAULT  = 12'd2700;  // > 2662 -> >85°C

    // ── State name decoder ──
    function [127:0] state_name;
        input [2:0] s;
        begin
            case(s)
                3'd0: state_name = "NORMAL     ";
                3'd1: state_name = "WARN       ";
                3'd2: state_name = "FAULT      ";
                3'd3: state_name = "SENSOR_FAIL";
                3'd4: state_name = "RECOVERY   ";
                default: state_name = "UNKNOWN    ";
            endcase
        end
    endfunction

    task print_status;
        input [127:0] label;
        begin
            $display("\n[%0t ms] %0s", $time, label);
            $display("  EXH=%0s  OIL=%0s  COOL=%0s  AIR=%0s  TURB=%0s  VIB=%0s  PLUM=%0s",
                state_name(state_exh_temp),  state_name(state_lube_oil),
                state_name(state_coolant),   state_name(state_start_air),
                state_name(state_turbo_oil), state_name(state_vibration),
                state_name(state_plummer));
            $display("  common_alarm=%b  common_shutdown=%b  engine_ready=%b  health=%0d%%  fail_mask=%07b",
                do_common_alarm, do_common_shutdown, do_engine_ready,
                engine_health_pct, sensor_fail_mask);
        end
    endtask

    initial begin
        $dumpfile("tb_engine_fsm_v2.vcd");
        $dumpvars(0, tb_engine_fsm_v2);

        // ── Initialise — all sensors normal ──
        rst_n = 0;
        exh_temp_adc     = EXH_NORMAL;
        lube_oil_adc     = OIL_NORMAL;
        coolant_adc      = COOL_NORMAL;
        start_air_adc    = AIR_NORMAL;
        turbo_oil_adc    = TURB_NORMAL;
        vibration_adc    = VIB_NORMAL;
        plummer_temp_adc = PLUM_NORMAL;
        emerg_stop = 0; overspeed = 0; prelube_ok = 0;
        alarm_ack  = 0; fault_cleared = 0;
        fmi_open_ckt = 0; fmi_short = 0;

        repeat(4) @(posedge clk);
        rst_n = 1;
        repeat(3) @(posedge clk);

        $display("════════════════════════════════════════════════════════");
        $display("  PORT MAIN ENGINE — 7-SENSOR FSM SIMULATION");
        $display("════════════════════════════════════════════════════════");

        // ── T1: Normal operation, all sensors healthy ──
        repeat(5) @(posedge clk);
        print_status("T1: Normal operation - all 7 sensors healthy");

        // ── T2: Exhaust temp injector fouling (Python day 65-70 event) ──
        $display("\n-- T2: Exhaust temp rises (injector fouling) --");
        exh_temp_adc = EXH_WARN;
        repeat(3) @(posedge clk);
        print_status("T2a: EXH_PORT_TEMP -> WARN (>580C)");

        exh_temp_adc = EXH_FAULT;
        repeat(55) @(posedge clk);  // wait for alarm delay to expire
        print_status("T2b: EXH_PORT_TEMP -> FAULT (>600C, alarm delay expired)");

        // Recover exhaust temp
        exh_temp_adc  = EXH_NORMAL;
        fault_cleared = 1;
        alarm_ack     = 1;
        @(posedge clk); @(posedge clk);
        prelube_ok = 1;
        repeat(3) @(posedge clk);
        print_status("T2c: EXH_PORT_TEMP recovered -> NORMAL");
        fault_cleared = 0; alarm_ack = 0; prelube_ok = 0;

        // ── T3: Lube oil pressure dip (Python day 55-57 event) ──
        $display("\n-- T3: Lube oil pressure dips (filter clog) --");
        lube_oil_adc = OIL_WARN;
        repeat(3) @(posedge clk);
        print_status("T3a: LUBE_OIL_PRES -> WARN (<150kPa)");

        lube_oil_adc = OIL_FAULT;
        repeat(55) @(posedge clk);
        print_status("T3b: LUBE_OIL_PRES -> FAULT (<100kPa)");

        lube_oil_adc  = OIL_NORMAL;
        fault_cleared = 1; alarm_ack = 1;
        @(posedge clk); @(posedge clk);
        prelube_ok = 1;
        repeat(3) @(posedge clk);
        print_status("T3c: LUBE_OIL_PRES recovered -> NORMAL");
        fault_cleared = 0; alarm_ack = 0; prelube_ok = 0;

        // ── T4: Vibration imbalance fault (Python day 50+ event) ──
        $display("\n-- T4: Vibration rises (rotor imbalance) --");
        vibration_adc = VIB_WARN;
        repeat(3) @(posedge clk);
        print_status("T4a: VIBRATION_RMS -> WARN (>9.0mm/s, ISO Zone C)");

        vibration_adc = VIB_FAULT;
        repeat(55) @(posedge clk);
        print_status("T4b: VIBRATION_RMS -> FAULT (>14.0mm/s, ISO Zone D)");

        vibration_adc = VIB_NORMAL;
        fault_cleared = 1; alarm_ack = 1;
        @(posedge clk); @(posedge clk);
        prelube_ok = 1;
        repeat(3) @(posedge clk);
        print_status("T4c: VIBRATION_RMS recovered -> NORMAL");
        fault_cleared = 0; alarm_ack = 0; prelube_ok = 0;

        // ── T5: Plummer bearing temp rise (Python day 60+ oil restriction) ──
        $display("\n-- T5: Plummer block bearing temp rises (oil restriction) --");
        plummer_temp_adc = PLUM_WARN;
        repeat(3) @(posedge clk);
        print_status("T5a: PLUMMER_BRG_TEMP -> WARN (>75C)");

        plummer_temp_adc = PLUM_FAULT;
        repeat(55) @(posedge clk);
        print_status("T5b: PLUMMER_BRG_TEMP -> FAULT (>85C)");

        plummer_temp_adc = PLUM_NORMAL;
        fault_cleared = 1; alarm_ack = 1;
        @(posedge clk); @(posedge clk);
        prelube_ok = 1;
        repeat(3) @(posedge clk);
        print_status("T5c: PLUMMER_BRG_TEMP recovered -> NORMAL");
        fault_cleared = 0; alarm_ack = 0; prelube_ok = 0;

        // ── T6: Multiple simultaneous faults — common_shutdown check ──
        $display("\n-- T6: Multiple simultaneous faults (worst case) --");
        exh_temp_adc  = EXH_FAULT;
        lube_oil_adc  = OIL_FAULT;
        vibration_adc = VIB_FAULT;
        repeat(55) @(posedge clk);
        print_status("T6: Triple fault -> common_shutdown should be HIGH");

        // ── T7: Full recovery — everything back to normal ──
        $display("\n-- T7: Full system recovery --");
        exh_temp_adc     = EXH_NORMAL;
        lube_oil_adc     = OIL_NORMAL;
        coolant_adc      = COOL_NORMAL;
        start_air_adc    = AIR_NORMAL;
        turbo_oil_adc    = TURB_NORMAL;
        vibration_adc    = VIB_NORMAL;
        plummer_temp_adc = PLUM_NORMAL;
        fault_cleared = 1; alarm_ack = 1;
        @(posedge clk); @(posedge clk);
        prelube_ok = 1;
        repeat(3) @(posedge clk);
        print_status("T7: Full recovery -> all NORMAL, engine_ready should be HIGH");

        $display("\n════════════════════════════════════════════════════════");
        $display("  SIMULATION COMPLETE");
        $display("════════════════════════════════════════════════════════");
        $finish;
    end

endmodule
