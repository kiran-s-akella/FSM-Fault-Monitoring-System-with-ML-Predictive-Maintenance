`timescale 1ms/1us

// ════════════════════════════════════════════════════════════════════════
// engine_fsm_v2.v
// PORT Main Engine — top-level FSM aggregator
// Instantiates one sensor_fsm per sensor used in the ML predictive
// maintenance pipeline (7 sensors total).
//
// Sensor list (matches SENSOR_CONFIG from Python pipeline):
//   1. EXH_PORT_TEMP    — Exhaust Port Temperature   (warn_hi/alarm_hi)
//   2. LUBE_OIL_PRES    — Lube Oil Pressure          (warn_lo/alarm_lo)
//   3. COOLANT_PRES     — Coolant Pressure           (warn_lo/alarm_lo)
//   4. START_AIR_PRES   — Starting Air Pressure      (warn_lo/alarm_lo)
//   5. TURBO_OIL_PRES   — Turbocharger Oil Pressure  (warn_lo/alarm_lo)
//   6. VIBRATION_RMS    — Crankshaft Vibration       (warn_hi/alarm_hi)
//   7. PLUMMER_BRG_TEMP — Plummer Block Bearing Temp (warn_hi/alarm_hi)
//
// Each sensor_fsm instance reuses the EXACT module from your Step-3
// simulation (sensor_fsm.v) — no changes needed there.
// ════════════════════════════════════════════════════════════════════════

module engine_fsm_v2 (
    input  wire        clk,
    input  wire        rst_n,

    // ── Raw 12-bit ADC inputs (0-4095) for each sensor ──
    input  wire [11:0] exh_temp_adc,
    input  wire [11:0] lube_oil_adc,
    input  wire [11:0] coolant_adc,
    input  wire [11:0] start_air_adc,
    input  wire [11:0] turbo_oil_adc,
    input  wire [11:0] vibration_adc,
    input  wire [11:0] plummer_temp_adc,

    // ── Common interlock inputs (shared across all sensors) ──
    input  wire        emerg_stop,     // hardwired E-stop
    input  wire        overspeed,      // overspeed trip
    input  wire        prelube_ok,     // prelube satisfied
    input  wire        alarm_ack,      // operator acknowledge
    input  wire        fault_cleared,  // fault condition removed

    // ── J1939 fault flags (shared bus, applies to whichever sensor reports it) ──
    input  wire        fmi_open_ckt,
    input  wire        fmi_short,

    // ── Outputs → PLC SDO coils ──
    output wire        do_common_alarm,     // any sensor in WARN or worse
    output wire        do_common_shutdown,  // any sensor in FAULT
    output wire        do_engine_ready,     // all sensors NORMAL + interlocks clear

    // ── Per-sensor diagnostic state outputs (3 bits each) ──
    output wire [2:0]  state_exh_temp,
    output wire [2:0]  state_lube_oil,
    output wire [2:0]  state_coolant,
    output wire [2:0]  state_start_air,
    output wire [2:0]  state_turbo_oil,
    output wire [2:0]  state_vibration,
    output wire [2:0]  state_plummer,

    // ── Sensor failure mask (1 bit per sensor) ──
    output wire [6:0]  sensor_fail_mask,

    // ── Engine health score (0-100, computed from per-sensor states) ──
    output wire [6:0]  engine_health_pct
);

    // Common interlock OR — drives srt_intck on all sensors
    wire srt_intck = emerg_stop | overspeed;

    // ── Per-sensor alarm/shutdown/fail wires ──
    wire alm_exh,  shd_exh,  fail_exh;
    wire alm_oil,  shd_oil,  fail_oil;
    wire alm_cool, shd_cool, fail_cool;
    wire alm_air,  shd_air,  fail_air;
    wire alm_turb, shd_turb, fail_turb;
    wire alm_vib,  shd_vib,  fail_vib;
    wire alm_plum, shd_plum, fail_plum;

    // ════════════════════════════════════════════════════════════════════
    // 1. EXH_PORT_TEMP — warn_hi=580°C(3112) alarm_hi=600°C(3276)
    //    range 200-700°C
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd01),
        .SENSOR_TYPE (2'b11),        // SRTD (RTD/thermocouple type)
        .ALARM_HI    (12'd3276),     // 600°C
        .ALARM_LO    (12'd0),
        .WARN_HI     (12'd3112),     // 580°C
        .WARN_LO     (12'd0)
    ) u_fsm_exh_temp (
        .clk(clk), .rst_n(rst_n),
        .raw_value(exh_temp_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_exh_temp),
        .alarm_out(alm_exh), .shutdown_out(shd_exh), .fail_out(fail_exh)
    );

    // ════════════════════════════════════════════════════════════════════
    // 2. LUBE_OIL_PRES — warn_lo=150kPa(1788) alarm_lo=100kPa(1459)
    //    range 0-500 kPa, 4-20mA mapping (ADC=800 + value/range*3295)
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd02),
        .SENSOR_TYPE (2'b10),        // SAI 4-20mA
        .ALARM_HI    (12'd4095),
        .ALARM_LO    (12'd1459),     // 100 kPa
        .WARN_HI     (12'd4095),
        .WARN_LO     (12'd1788)      // 150 kPa
    ) u_fsm_lube_oil (
        .clk(clk), .rst_n(rst_n),
        .raw_value(lube_oil_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_lube_oil),
        .alarm_out(alm_oil), .shutdown_out(shd_oil), .fail_out(fail_oil)
    );

    // ════════════════════════════════════════════════════════════════════
    // 3. COOLANT_PRES — warn_lo=50kPa(1898) alarm_lo=15kPa(1130)
    //    range 0-150 kPa, 4-20mA mapping
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd03),
        .SENSOR_TYPE (2'b10),
        .ALARM_HI    (12'd4095),
        .ALARM_LO    (12'd1130),     // 15 kPa
        .WARN_HI     (12'd4095),
        .WARN_LO     (12'd1898)      // 50 kPa
    ) u_fsm_coolant (
        .clk(clk), .rst_n(rst_n),
        .raw_value(coolant_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_coolant),
        .alarm_out(alm_cool), .shutdown_out(shd_cool), .fail_out(fail_cool)
    );

    // ════════════════════════════════════════════════════════════════════
    // 4. START_AIR_PRES — warn_lo=800kPa(2351) alarm_lo=500kPa(1769)
    //    range 0-1700 kPa, 4-20mA mapping
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd04),
        .SENSOR_TYPE (2'b10),
        .ALARM_HI    (12'd4095),
        .ALARM_LO    (12'd1769),     // 500 kPa
        .WARN_HI     (12'd4095),
        .WARN_LO     (12'd2351)      // 800 kPa
    ) u_fsm_start_air (
        .clk(clk), .rst_n(rst_n),
        .raw_value(start_air_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_start_air),
        .alarm_out(alm_air), .shutdown_out(shd_air), .fail_out(fail_air)
    );

    // ════════════════════════════════════════════════════════════════════
    // 5. TURBO_OIL_PRES — warn_lo=60kPa(965) alarm_lo=30kPa(882)
    //    range 0-1200 kPa, 4-20mA mapping
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd05),
        .SENSOR_TYPE (2'b10),
        .ALARM_HI    (12'd4095),
        .ALARM_LO    (12'd882),      // 30 kPa
        .WARN_HI     (12'd4095),
        .WARN_LO     (12'd965)       // 60 kPa
    ) u_fsm_turbo_oil (
        .clk(clk), .rst_n(rst_n),
        .raw_value(turbo_oil_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_turbo_oil),
        .alarm_out(alm_turb), .shutdown_out(shd_turb), .fail_out(fail_turb)
    );

    // ════════════════════════════════════════════════════════════════════
    // 6. VIBRATION_RMS — warn_hi=9.0mm/s(1986) alarm_hi=14.0mm/s(2645)
    //    range 0-25 mm/s, 4-20mA mapping   (NEW SENSOR)
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd06),
        .SENSOR_TYPE (2'b10),        // SAI from proximity probe conditioner
        .ALARM_HI    (12'd2645),     // 14.0 mm/s — ISO Zone D
        .ALARM_LO    (12'd0),
        .WARN_HI     (12'd1986),     // 9.0 mm/s — ISO Zone C
        .WARN_LO     (12'd0)
    ) u_fsm_vibration (
        .clk(clk), .rst_n(rst_n),
        .raw_value(vibration_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_vibration),
        .alarm_out(alm_vib), .shutdown_out(shd_vib), .fail_out(fail_vib)
    );

    // ════════════════════════════════════════════════════════════════════
    // 7. PLUMMER_BRG_TEMP — warn_hi=75°C(2252) alarm_hi=85°C(2662)
    //    range 20-120°C   (NEW SENSOR)
    // ════════════════════════════════════════════════════════════════════
    sensor_fsm #(
        .SENSOR_ID   (8'd07),
        .SENSOR_TYPE (2'b11),        // SRTD — PT100
        .ALARM_HI    (12'd2662),     // 85°C
        .ALARM_LO    (12'd0),
        .WARN_HI     (12'd2252),     // 75°C
        .WARN_LO     (12'd0)
    ) u_fsm_plummer (
        .clk(clk), .rst_n(rst_n),
        .raw_value(plummer_temp_adc), .value_valid(1'b1),
        .emerg_stop(emerg_stop), .overspeed(overspeed), .srt_intck(srt_intck),
        .fmi_open_ckt(fmi_open_ckt), .fmi_short(fmi_short),
        .prelube_ok(prelube_ok), .alarm_ack(alarm_ack), .fault_cleared(fault_cleared),
        .state(state_plummer),
        .alarm_out(alm_plum), .shutdown_out(shd_plum), .fail_out(fail_plum)
    );

    // ════════════════════════════════════════════════════════════════════
    // Aggregate outputs
    // ════════════════════════════════════════════════════════════════════
    assign do_common_alarm = alm_exh | alm_oil | alm_cool | alm_air
                            | alm_turb | alm_vib | alm_plum;

    assign do_common_shutdown = shd_exh | shd_oil | shd_cool | shd_air
                               | shd_turb | shd_vib | shd_plum;

    assign do_engine_ready = (state_exh_temp  == 3'd0)
                            & (state_lube_oil  == 3'd0)
                            & (state_coolant   == 3'd0)
                            & (state_start_air == 3'd0)
                            & (state_turbo_oil == 3'd0)
                            & (state_vibration == 3'd0)
                            & (state_plummer   == 3'd0)
                            & ~emerg_stop
                            & prelube_ok;

    assign sensor_fail_mask = {fail_plum, fail_vib, fail_turb,
                                fail_air, fail_cool, fail_oil, fail_exh};

    // ════════════════════════════════════════════════════════════════════
    // Health score approximation (matches Python weighted scheme)
    // Each sensor contributes its weight (in percentage points, integer)
    // ONLY if its state == NORMAL (3'd0). Anything else contributes 0.
    // Weights (×100 to keep integer): EXH=25 OIL=20 COOL=15 AIR=10
    //                                  TURB=10 VIB=12 PLUM=8   (sum=100)
    //
    // This is a simplified hardware proxy for the Python health score —
    // full continuous scoring is done in software; this gives a quick
    // discrete sanity-check value visible on the FPGA/sim side.
    // ════════════════════════════════════════════════════════════════════
    wire [6:0] health_exh  = (state_exh_temp  == 3'd0) ? 7'd25 : 7'd0;
    wire [6:0] health_oil  = (state_lube_oil  == 3'd0) ? 7'd20 : 7'd0;
    wire [6:0] health_cool = (state_coolant   == 3'd0) ? 7'd15 : 7'd0;
    wire [6:0] health_air  = (state_start_air == 3'd0) ? 7'd10 : 7'd0;
    wire [6:0] health_turb = (state_turbo_oil == 3'd0) ? 7'd10 : 7'd0;
    wire [6:0] health_vib  = (state_vibration == 3'd0) ? 7'd12 : 7'd0;
    wire [6:0] health_plum = (state_plummer   == 3'd0) ? 7'd8  : 7'd0;

    assign engine_health_pct = health_exh + health_oil + health_cool
                              + health_air + health_turb + health_vib
                              + health_plum;

endmodule
