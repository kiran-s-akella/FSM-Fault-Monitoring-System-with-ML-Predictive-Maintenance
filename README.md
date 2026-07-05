# FSM-Fault-Monitoring-System-with-ML-Predictive-Maintenance

An end-to-end predictive maintenance framework developed for a ship's **PORT Main Engine**, utilizing data constraints from the standard engine safety PLC Input/Output list[cite: 1].

## Features
* **ML Track (Google Colab):** Performs multi-variate anomaly detection via Isolation Forest, tracks continuous system degradation, and models sensor trends over a 30-day lookahead horizon using XGBoost[cite: 1].
* **RTL Track (Xilinx Vivado):** Implement an independent, hardware-verifiable 4-state Moore Machine safety interlock layer to provide quick checking references against your ML calculations[cite: 1].

## Repository Content
* `ml_pipeline/engine_predictive_maintenance.ipynb`: The end-to-end data pipeline script[cite: 1].
* `rtl/engine_fsm.v`: Synthesizable state transition safety block[cite: 1].
* `rtl/tb_engine_fsm.v`: Functional validation verification suite[cite: 1].
