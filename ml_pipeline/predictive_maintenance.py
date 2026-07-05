# -*- coding: utf-8 -*-
"""Engine Health Monitoring Pipeline"""

!pip install xgboost prophet --quiet

import warnings
from datetime import datetime
import matplotlib.dates as mdates
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from sklearn.ensemble import IsolationForest
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import MinMaxScaler, StandardScaler
import xgboost as xgb

warnings.filterwarnings('ignore')

# ──────────────────────────────────────────────────────────────────────────────
# 1. CONFIGURATIONS & STYLING
# ──────────────────────────────────────────────────────────────────────────────

plt.rcParams.update({
    'figure.facecolor': '#0a0f1e', 'axes.facecolor': '#0d1526', 'axes.edgecolor': '#2a3f5f',
    'axes.labelcolor': '#c8d8e8', 'xtick.color': '#8899aa', 'ytick.color': '#8899aa',
    'text.color': '#c8d8e8', 'grid.color': '#1e2d45', 'grid.linestyle': '--',
    'grid.alpha': 0.5, 'font.family': 'monospace', 'figure.dpi': 120,
})

COLORS = {
    'normal': '#00d4aa', 'warn': '#f0a500', 'fault': '#e83f3f',
    'recovery': '#4a9eff', 'health': '#00d4aa', 'predict': '#a855f7', 'threshold': '#ff6b35',
}

SENSOR_CONFIG = {
    'EXH_PORT_TEMP':   {'range_min': 200, 'range_max': 700, 'warn_hi': 580,  'alarm_hi': 600,  'warn_lo': None, 'alarm_lo': None, 'normal_op': (400, 560), 'weight': 0.25,  'description': 'Mean Exhaust Port Temperature'},
    'LUBE_OIL_PRES':   {'range_min': 0,   'range_max': 500, 'warn_hi': None, 'alarm_hi': None, 'warn_lo': 150,  'alarm_lo': 100,  'normal_op': (260, 400), 'weight': 0.20,  'description': 'Lube Oil Pressure'},
    'COOLANT_PRES':    {'range_min': 0,   'range_max': 150, 'warn_hi': None, 'alarm_hi': None, 'warn_lo': 50,   'alarm_lo': 15,   'normal_op': (70, 130),  'weight': 0.15,  'description': 'Engine Coolant Pressure'},
    'START_AIR_PRES':  {'range_min': 0,   'range_max': 1700,'warn_hi': None, 'alarm_hi': None, 'warn_lo': 800,  'alarm_lo': 500,  'normal_op': (1000, 1600),'weight': 0.10, 'description': 'Starting Air Pressure'},
    'TURBO_OIL_PRES':  {'range_min': 0,   'range_max': 1200,'warn_hi': None, 'alarm_hi': None, 'warn_lo': 60,   'alarm_lo': 30,   'normal_op': (150, 400), 'weight': 0.10,  'description': 'Turbocharger Oil Inlet Pressure'},
    'VIBRATION_RMS':   {'range_min': 0.0, 'range_max': 25.0,'warn_hi': 9.0,  'alarm_hi': 14.0, 'warn_lo': None, 'alarm_lo': None, 'normal_op': (0.5, 6.0),  'weight': 0.12,  'description': 'Crankshaft Vibration RMS'},
    'PLUMMER_BRG_TEMP':{'range_min': 20.0,'range_max': 120.0,'warn_hi': 75.0, 'alarm_hi': 85.0, 'warn_lo': None, 'alarm_lo': None, 'normal_op': (35.0, 65.0), 'weight': 0.08,  'description': 'Plummer Block Temperature'},
}

# ──────────────────────────────────────────────────────────────────────────────
# 2. DATA INGESTION & PREPROCESSING
# ──────────────────────────────────────────────────────────────────────────────

np.random.seed(42)

def generate_realistic_engine_data(days=90, freq_minutes=1):
    n_points = days * 24 * 60 // freq_minutes
    timestamps = pd.date_range(start='2024-01-01', periods=n_points, freq=f'{freq_minutes}min')
    t_hours = np.arange(n_points) * freq_minutes / 60

    exh_temp = 480 + 15 * np.sin(2 * np.pi * t_hours / 24 - np.pi/4) + np.linspace(0, 20, n_points) + np.random.normal(0, 3, n_points)
    exh_temp[int(65*24*60):int(70*24*60)] += np.linspace(0, 95, int(5*24*60))
    exh_temp[int(40*24*60):int(40*24*60)+180] += 40

    lube_oil = 310 + np.linspace(0, -60, n_points) + np.random.normal(0, 5, n_points)
    lube_oil[int(55*24*60):int(57*24*60)] -= np.linspace(0, 80, int(2*24*60))

    coolant_pres = 95 + np.linspace(0, -15, n_points) + np.random.normal(0, 2, n_points)
    coolant_pres[int(75*24*60):] -= np.linspace(0, 35, n_points - int(75*24*60))

    start_air = 1200 + 80 * np.sin(2 * np.pi * t_hours / 8) + np.random.normal(0, 20, n_points)
    start_air[int(80*24*60):] -= np.linspace(0, 500, n_points - int(80*24*60))

    turbo_oil = 220 + np.linspace(0, -30, n_points) + np.random.normal(0, 4, n_points)
    vibration = 3.5 + np.linspace(0, 0.5, n_points) + np.abs(np.random.normal(0, 0.3, n_points)) + 0.8 * np.sin(2 * np.pi * t_hours / 24)
    vibration[int(50*24*60):] += np.linspace(0, 0.1, n_points - int(50*24*60))
    vibration[int(30*24*60):int(30*24*60)+240] += 0.1 * np.abs(np.sin(np.linspace(0, np.pi, 240)))

    plummer_temp = 48.0 + np.linspace(0, 1.0, n_points) + np.random.normal(0, 0.8, n_points) + 2.5 * np.sin(2 * np.pi * t_hours / 24)
    plummer_temp[int(60*24*60):] += np.linspace(0, 0.5, n_points - int(60*24*60))
    plummer_temp[int(45*24*60):int(45*24*60)+360] += 0.5

    engine_rpm = np.clip(900 + np.random.normal(0, 20, n_points), 300, 1200)

    return pd.DataFrame({
        'EXH_PORT_TEMP': np.clip(exh_temp, 200, 700), 'LUBE_OIL_PRES': np.clip(lube_oil, 0, 500),
        'COOLANT_PRES': np.clip(coolant_pres, 0, 150), 'START_AIR_PRES': np.clip(start_air, 0, 1700),
        'TURBO_OIL_PRES': np.clip(turbo_oil, 0, 1200), 'VIBRATION_RMS': np.clip(vibration, 0.0, 25.0),
        'PLUMMER_BRG_TEMP': np.clip(plummer_temp, 20.0, 120.0), 'ENGINE_RPM': engine_rpm
    }, index=timestamps)

df_raw = generate_realistic_engine_data()
resample_dict = {
    'EXH_PORT_TEMP': 'mean', 'LUBE_OIL_PRES': 'mean', 'COOLANT_PRES': 'mean',
    'START_AIR_PRES': 'mean', 'TURBO_OIL_PRES': 'mean', 'VIBRATION_RMS': 'max',
    'PLUMMER_BRG_TEMP': 'mean', 'ENGINE_RPM': 'mean'
}
df_hourly = df_raw.resample('1h').agg(resample_dict).round(3).ffill(limit=2).bfill(limit=2)
sensor_cols = list(SENSOR_CONFIG.keys())

df_scaled = df_hourly.copy()
df_scaled[sensor_cols] = MinMaxScaler().fit_transform(df_hourly[sensor_cols])

# ──────────────────────────────────────────────────────────────────────────────
# 3. FEATURE ENGINEERING
# ──────────────────────────────────────────────────────────────────────────────

def engineer_features(df, sensor_config):
    fe = df.copy()
    for col, cfg in sensor_config.items():
        fe[f'{col}_roll24_mean'] = fe[col].rolling(24, min_periods=1).mean()
        fe[f'{col}_roll24_std']  = fe[col].rolling(24, min_periods=1).std().fillna(0)
        fe[f'{col}_roll6_mean']  = fe[col].rolling(6,  min_periods=1).mean()
        fe[f'{col}_lag1'] = fe[col].shift(1)
        fe[f'{col}_lag6'] = fe[col].shift(6)
        fe[f'{col}_lag24'] = fe[col].shift(24)
        fe[f'{col}_roc'] = fe[col].diff(1)
        
        dev = pd.Series(0.0, index=fe.index)
        if cfg['warn_hi'] is not None: dev += (fe[col] - cfg['warn_hi']).clip(lower=0)
        if cfg['warn_lo'] is not None: dev += (cfg['warn_lo'] - fe[col]).clip(lower=0)
        fe[f'{col}_deviation'] = dev

    fe['hour_of_day'], fe['day_of_week'] = fe.index.hour, fe.index.dayofweek
    return fe.bfill().fillna(0)

df_features = engineer_features(df_hourly, SENSOR_CONFIG)

# ──────────────────────────────────────────────────────────────────────────────
# 4. ANOMALY DETECTION (ISOLATION FOREST)
# ──────────────────────────────────────────────────────────────────────────────

anomaly_features = sensor_cols + [f'{c}_roll24_mean' for c in sensor_cols] + \
                   [f'{c}_roll24_std' for c in sensor_cols] + \
                   [f'{c}_deviation' for c in sensor_cols] + [f'{c}_roc' for c in sensor_cols]

X_anomaly_scaled = StandardScaler().fit_transform(df_features[anomaly_features].values)
iso_forest = IsolationForest(n_estimators=200, contamination=0.05, random_state=42, n_jobs=-1)
iso_forest.fit(X_anomaly_scaled)

anomaly_scores = iso_forest.decision_function(X_anomaly_scaled)
anomaly_score_norm = 1 - (anomaly_scores - anomaly_scores.min()) / (anomaly_scores.max() - anomaly_scores.min())

df_hourly['anomaly_score'] = anomaly_score_norm
df_hourly['is_anomaly'] = (iso_forest.predict(X_anomaly_scaled) == -1).astype(int)
df_features['anomaly_score'], df_features['is_anomaly'] = df_hourly['anomaly_score'], df_hourly['is_anomaly']

# ──────────────────────────────────────────────────────────────────────────────
# 5. HEALTH SCORING SYSTEM
# ──────────────────────────────────────────────────────────────────────────────

def compute_sensor_health(value, cfg):
    lo_alarm, lo_warn, hi_warn, hi_alarm = cfg.get('alarm_lo'), cfg.get('warn_lo'), cfg.get('warn_hi'), cfg.get('alarm_hi')
    norm_lo, norm_hi = cfg['normal_op']
    mid, band_half = (norm_lo + norm_hi) / 2, (norm_hi - norm_lo) / 2

    if norm_lo <= value <= norm_hi:
        return 100 - (abs(value - mid) / band_half) * 20
    if hi_warn is not None and hi_alarm is not None and value > norm_hi:
        return (80 - ((value - norm_hi) / (hi_warn - norm_hi)) * 20) if value <= hi_warn else \
               (60 - ((value - hi_warn) / (hi_alarm - hi_warn)) * 50) if value <= hi_alarm else \
               max(0, 10 - min((value - hi_alarm) / hi_alarm, 1.0) * 10)
    if lo_warn is not None and lo_alarm is not None and value < norm_lo:
        return (80 - ((norm_lo - value) / (norm_lo - lo_warn)) * 20) if value >= lo_warn else \
               (60 - ((lo_warn - value) / (lo_warn - lo_alarm)) * 50) if value >= lo_alarm else \
               max(0, 10 - min((lo_alarm - value) / lo_alarm, 1.0) * 10)
    return 75

for col, cfg in SENSOR_CONFIG.items():
    df_hourly[f'{col}_health'] = df_hourly[col].apply(lambda v: compute_sensor_health(v, cfg))

health_cols = [f'{c}_health' for c in sensor_cols]
weights = np.array([SENSOR_CONFIG[c]['weight'] for c in sensor_cols])
df_hourly['engine_health'] = (df_hourly[health_cols].values @ weights - df_hourly['anomaly_score'] * 15).clip(0, 100)

# ──────────────────────────────────────────────────────────────────────────────
# 6. XGBOOST PREDICTIVE FORECASTING
# ──────────────────────────────────────────────────────────────────────────────

FORECAST_HOURS, N_LAGS = 30 * 24, 48

def train_xgb_forecaster(series, n_lags=48):
    values = series.values
    X, y = [], []
    for i in range(n_lags, len(values)):
        X.append(values[i-n_lags:i])
        y.append(values[i])
    X, y = np.array(X), np.array(y)
    split = int(len(X) * 0.8)
    model = xgb.XGBRegressor(n_estimators=300, learning_rate=0.05, max_depth=4, subsample=0.8, colsample_bytree=0.8, reg_alpha=0.1, reg_lambda=1.0, random_state=42, verbosity=0)
    model.fit(X[:split], y[:split], eval_set=[(X[split:], y[split:])], verbose=False)
    rmse = np.sqrt(mean_squared_error(y[split:], model.predict(X[split:])))
    return model, rmse

def forecast_recursive(model, history, n_steps, n_lags):
    predictions = []
    window = list(history[-n_lags:])
    for _ in range(n_steps):
        pred = model.predict(np.array(window[-n_lags:]).reshape(1, -1))[0]
        predictions.append(pred)
        window.append(pred)
    return np.array(predictions)

forecasts, rmse_scores = {}, {}
target_series = {col: df_hourly[col] for col in sensor_cols}
target_series['engine_health'] = df_hourly['engine_health']

for name, series in target_series.items():
    model, rmse = train_xgb_forecaster(series, n_lags=N_LAGS)
    forecasts[name] = forecast_recursive(model, series.values, FORECAST_HOURS, N_LAGS)
    rmse_scores[name] = rmse

future_index = pd.date_range(start=df_hourly.index[-1] + pd.Timedelta(hours=1), periods=FORECAST_HOURS, freq='1h')
df_forecast = pd.DataFrame(forecasts, index=future_index)

# ──────────────────────────────────────────────────────────────────────────────
# 7. RUL ESTIMATION & SCHEDULING
# ──────────────────────────────────────────────────────────────────────────────

def estimate_rul(forecast_vals, threshold, direction='below'):
    for i, v in enumerate(forecast_vals):
        if (direction == 'below' and v < threshold) or (direction == 'above' and v > threshold): return i
    return None

rul_estimates = {}
for col in sensor_cols:
    cfg = SENSOR_CONFIG[col]
    rul_estimates[col] = {
        'warn_hi': estimate_rul(forecasts[col], cfg['warn_hi'], 'above') if cfg['warn_hi'] else None,
        'alarm_hi': estimate_rul(forecasts[col], cfg['alarm_hi'], 'above') if cfg['alarm_hi'] else None,
        'warn_lo': estimate_rul(forecasts[col], cfg['warn_lo'], 'below') if cfg['warn_lo'] else None,
        'alarm_lo': estimate_rul(forecasts[col], cfg['alarm_lo'], 'below') if cfg['alarm_lo'] else None,
    }

print("✅ Pipeline executed successfully.")
print(f"Current Engine Health Score: {df_hourly['engine_health'].iloc[-1]:.2f}%")
