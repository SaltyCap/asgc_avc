import pandas as pd
import numpy as np
import glob
import os
from sklearn.preprocessing import MinMaxScaler

def load_data(log_dir):
    """
    Load all CSV files from the log directory.
    """
    all_files = glob.glob(os.path.join(log_dir, "*.csv"))
    df_list = []
    
    for filename in all_files:
        try:
            df = pd.read_csv(filename, index_col=None, header=0)
            df_list.append(df)
        except pd.errors.EmptyDataError:
            print(f"Skipping empty file: {filename}")
            continue

    if not df_list:
        print("No data found.")
        return None

    combined_df = pd.concat(df_list, axis=0, ignore_index=True)
    return combined_df

def preprocess_data(df):
    """
    Clean and normalize data for training.
    """
    if 'mode' not in df.columns:
        print("Missing column: mode")
        return None, None, None

    # Drop rows where mode is IDLE.
    # Supports both string labels ("IDLE") and numeric mode IDs (0).
    mode_series = df['mode']
    try:
        mode_is_numeric = np.issubdtype(mode_series.dtype, np.number)
    except TypeError:
        mode_is_numeric = False
    if mode_is_numeric:
        df = df[mode_series != 0]
    else:
        df = df[mode_series.astype(str).str.strip().str.upper() != 'IDLE']
    
    if df.empty:
        print("No active data found after filtering IDLE.")
        return None, None, None

    # Feature selection
    # Inputs: 
    # - target_l, actual_l (error is implicitly target - actual)
    # - target_r, actual_r
    # - gyro_z (to detect turning)
    # - odom_heading (maybe helpful, but relative)
    
    # We want to predict PWM (pulse_l, pulse_r)
    
    # Normalize inputs
    # Target and Actual are encoder counts (large ints)
    # Gyro is float
    
    feature_cols = ['target_l', 'actual_l', 'target_r', 'actual_r', 'gyro_z']
    if 'pulse_l' in df.columns and 'pulse_r' in df.columns:
        target_cols = ['pulse_l', 'pulse_r']
    elif 'pwm_l' in df.columns and 'pwm_r' in df.columns:
        target_cols = ['pwm_l', 'pwm_r']
    else:
        print("Missing target columns: expected pulse_l/pulse_r or pwm_l/pwm_r")
        return None, None, None
    
    # Ensure columns exist
    for col in feature_cols + target_cols:
        if col not in df.columns:
            print(f"Missing column: {col}")
            return None, None, None

    numeric_df = df[feature_cols + target_cols].apply(pd.to_numeric, errors='coerce').dropna()
    if numeric_df.empty:
        print("No valid numeric rows after cleaning.")
        return None, None, None

    X = numeric_df[feature_cols].values
    y = numeric_df[target_cols].values
    
    # Normalize features to 0-1 range
    scaler_X = MinMaxScaler()
    X_scaled = scaler_X.fit_transform(X)
    
    # Normalize targets (PWM) to 0-1 range or -1 to 1?
    # Pulse width is typically 1000000 to 2000000 ns
    # Center is 1500000
    scaler_y = MinMaxScaler()
    y_scaled = scaler_y.fit_transform(y)
    
    return X_scaled, y_scaled, (scaler_X, scaler_y)

if __name__ == "__main__":
    # Test run
    log_dir = "../logs"
    print(f"Loading data from {log_dir}...")
    df = load_data(log_dir)
    
    if df is not None:
        print(f"Loaded {len(df)} rows.")
        X, y, scalers = preprocess_data(df)
        if X is not None:
            print(f"Processed data shape: X={X.shape}, y={y.shape}")
        else:
            print("Preprocessing failed.")
