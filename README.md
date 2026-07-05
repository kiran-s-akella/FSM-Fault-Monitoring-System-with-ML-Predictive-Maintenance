# FSM Fault Monitoring System with ML Predictive Maintenance

A predictive maintenance framework for a vessel's **PORT Main Engine**, this project bridges the gap between sensor threshold alarms and predictive scheduling by deploying a two-track validation architecture: a data-driven Machine Learning pipeline and a hardware-verifiable Finite State Machine (FSM).

---

## 📋 Table of Contents
* **System Objectives**
* **1. Machine Learning Predictive Maintenance Pipeline**
* **2. Verilog Finite State Machine (FSM)**
* **Final Validation & Conclusion**


---



---

## 🎯 System Objectives

* **O1:** Identify multi-parametric anomalies using unsupervised machine learning.
* **O2:** Map real-time measurements into continuous 0–100% engine health scores.
* **O3:** Project sensor trajectories over a 30-day forecast window using recursive time-series modeling.
* **O4:** Generate a prioritized prescriptive maintenance schedule based on Remaining Useful Life (RUL) estimates.
* **O5:** Design, simulate, and evaluate a synthesizable RTL Finite State Machine representing deterministic engine health logic.

---

## 📊 1: Machine Learning Predictive Maintenance Pipeline 

Implemented in Python on Google Colab, this track handles time-series ingestion, feature engineering, trend forecasting, and automated maintenance planning.

### Pipeline Breakdown
1. **Data Preparation & Feature Engineering:** Hourly resampled dataset generated using forward/backward fills. Computes 24-hour rolling means and variances to filter high-frequency diurnal engine noise and expose underlying degradation vectors.
2. **Anomaly Detection (O1):** Uses an **Isolation Forest** model (200 estimators, 5% contamination) to evaluate multi-sensor coupling states and flag un-labeled statistical deviations.
3. **Health Scoring System (O2):** Employs a physics-informed scoring function mapping sensor coordinates against OEM operating limits into a 0–100% engine health index, penalized heavily by active anomaly scores.
4. **Predictive Prognostics (O3):** Deploys an **XGBoost Regressor** with a 48-hour lag window to compute a multi-step recursive 30-day forecast. 
5. **Prescriptive Scheduling (O4):** Multiplies RUL projections by a 20% safety margin to output a planned maintenance event timeline containing parts lists (e.g., fuel injector sets) and labor hour allocations.

### Simulation Results


* Identified **108 anomalous hours** (5.0% of a 90-day dataset) focused heavily around simulated injector fouling and oil filter clogging windows.
  
<img width="982" height="1010" alt="image" src="https://github.com/user-attachments/assets/7b9b62e6-a7f8-416e-bd2b-4a8add28a329" />


* Tracked continuous health decline from an initial **90% (GOOD band)** to a terminal **63.2% (MONITOR band)**.
  
<img width="970" height="822" alt="image" src="https://github.com/user-attachments/assets/c48e62b8-73a0-4a67-8406-85899aadc63e" />


* Predicted a warning boundary breach for the Exhaust Port Temperature in **4.3 days**, outputting a targeted inspection item scheduled 3.4 days out.

<img width="938" height="768" alt="image" src="https://github.com/user-attachments/assets/b602bc7c-1ebc-4007-9778-6e8e90ea9019" />


---

## ⚡ 2: Verilog Finite State Machine (FSM) 

The structural backbone of the safety-interlock layer is a hardware-equivalent Moore-type Finite State Machine designed in Verilog HDL. It translates discrete health condition flags across 7 critical sensors into high-reliability system states, providing an instantaneous backup checking layer.

### 1. Sensor Selection Architecture
Seven safety-critical parameters were isolated out of over 130 PLC signal channels based on engineering severity and correlation matrices:

| Sensor Subsystem | Signal Type | Normal Range | Warn Limit | Alarm Limit | Weight |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Exhaust Port Temp.** | RTD (°C) | 400–560 | 580 (hi) | 600 (hi) | 25% |
| **Lube Oil Pressure** | 4-20mA (kPa) | 260–400 | 150 (lo) | 100 (lo) | 20% |
| **Coolant Pressure** | 4-20mA (kPa) | 70–130 | 50 (lo) | 15 (lo) | 15% |
| **Starting Air Pressure** | 4-20mA (kPa) | 1000–1600 | 800 (lo) | 500 (lo) | 10% |
| **Turbocharger Oil Pres.** | 4-20mA (kPa) | 150–400 | 60 (lo) | 30 (lo) | 10% |
| **Crankshaft Vibration** | RMS (mm/s) | 0.5–6.0 | 9.0 (hi) | 14.0 (hi) | 12% |
| **Plummer Block Brg. Temp.**| RTD (°C) | 35–65 | 75 (hi) | 85 (hi) | 8% |

### 2. State Machine Logic & State Diagram

<img width="700" height="266" alt="image" src="https://github.com/user-attachments/assets/448e1b65-537f-449a-b48e-2f1e7a1c6e16" />

The FSM is parameterized using 2-bit state registers to enforce a four-tiered structural hierarchy:
* **`GOOD (2'b00)`:** Default initialization. Validated when all 7 active sensors are registering in their safe `OK` bands.
* **`MONITOR (2'b01)`:** Stepped up dynamically if **exactly one** sensor drops or rises into its respective `WARN` threshold.
* **`WARNING (2'b10)`:** Triggered automatically when **two or more** sensors simultaneously flag active `WARN` bounds.
* **`CRITICAL (2'b11)`:** The protective override loop. If **any single sensor** encounters an `ALARM` condition, the FSM instantly executes a direct shortcut bypass to `CRITICAL`, discarding intermediate counter states.

### 3. Simulation Verification (Xilinx Vivado)

<img width="686" height="520" alt="image" src="https://github.com/user-attachments/assets/ca3f1d11-f975-4337-9796-1fa3211ffe35" />

<img width="722" height="472" alt="image" src="https://github.com/user-attachments/assets/091a1bec-edbe-42ac-8028-e765f1f27101" />


The module's behavior was validated by applying five sequential, discrete test environments ($T1 \rightarrow T5$) directly to the sensor input vectors within a dedicated testbench:

* **T1 (All Sensors OK):** System outputs `engine_state = GOOD (00)`, `alarm_led = 0`, `shutdown_led = 0`.
* **T2 (Exhaust Temp $\rightarrow$ WARN):** Evaluates internal counter loops. System increments sequentially to `MONITOR (01)`; lines remain low.
* **T3 (+ Vibration $\rightarrow$ WARN):** Internal loop registers multiple warnings. State drives to `WARNING (10)`, firing `alarm_led = 1`.
* **T4 (+ Lube Oil $\rightarrow$ ALARM):** Simulates catastrophic pressure loss. The FSM drops out of warning count subroutines instantly to assert `CRITICAL (11)`, firing `alarm_led = 1` and `shutdown_led = 1`.
* **T5 (Full Recovery Applied):** Demonstrates the FSM retains zero latching histories, successfully resetting all status registers directly back to `GOOD (00)`.

---

## ✅ Final Validation & Conclusion
The framework pairs two structural logic layers to validate shipboard operations:
* **The ML Pipeline (Weighted Average):** Optimal for long-term health trending and predictive scheduling since isolated sensor drops are moderated by adjacent healthy parameters.
* **The Verilog FSM (Worst-Case Logic):** Guarantees high safety because a single lethal fault (e.g., structural breakdown of the lubrication line) is immediately acted upon and can never be "averaged away".

Evaluating discrepancies between the ML score bounds and the hardware FSM states gives instant insights into whether engine issues are due to single-component failure modes or the full system .

---

## 📋 Report
