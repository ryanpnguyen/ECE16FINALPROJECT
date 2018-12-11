"""
This function uses the STD of the data to calculate the heart rate, from the ECG data.

INPUT:
    ecg_signal: (2, N) numpy ndarray
                [0, :] contains the timestamps, in seconds (they DO NOT need to start at 0)
                [1, :] contains the raw (0-1023) ECG values

OUTPUT:
    HR: float, heart rate
    beat_loc: (N) numpy ndarray, contains hearbeat locations
    
"""

import numpy as np

def calculate_hr(ecg_signal):
    
    # Variable to contain the heartbeat locations
#    print(ecg_signal.shape)
    hb_loc = np.zeros((np.size(ecg_signal, 0), np.size(ecg_signal, 1)))

    # Calculate signal statistics and choose an adaptive threshold
    meanval = np.mean(ecg_signal[1,:])
    stdval = np.std(ecg_signal[1,:])
    thresh = meanval + 0.09*stdval

    # Find beginning of beat_loc (count as heartbeat) by looking at the difference operator (approximate gradient)
    beat_loc = ecg_signal[1,:] > thresh
    beat_loc_flip = -(beat_loc-1)
    hb_loc[1, 1:] = beat_loc[1:]*beat_loc_flip[:-1]
    hb_loc[0, :] = ecg_signal[0, :]

    # Calculate the HR by looking at the time difference between successive heartbeats
    hb_times = hb_loc[0, np.where(hb_loc[1, 1:] == 1)][0]
    hb_time_dif = hb_times[1:] - hb_times[:-1]
    hr_at_hb = 60 / hb_time_dif
    
    # Set HR as the average HR over this window of data
    HR = np.mean(hr_at_hb)
    
    print(HR)

    return HR, beat_loc * np.max(ecg_signal[1, :])