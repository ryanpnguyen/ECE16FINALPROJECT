# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 15:21:29 2018

@author: Ryan
"""
import serial, sys
from time import sleep
import time
from hr_ecg import calculate_hr

# ==================== Imports ====================
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import math
import filtering 
import scipy.signal as sig

lp_cutoff = 10
hp_cutoff = 2
filter_order = 3
SF = 200

filter_ICs = np.zeros((2, filter_order))
filter_ICs_IR = np.zeros((2, filter_order))

lp_num, lp_denom = sig.butter(filter_order, lp_cutoff/(SF/2), btype='lowpass', analog=False, output='ba')
hp_num, hp_denom = sig.butter(filter_order, hp_cutoff/(SF/2), btype='highpass', analog=False, output='ba')

filter_coeffs = np.zeros((4,filter_order + 1))

filter_coeffs[0][:] = hp_denom
filter_coeffs[1][:] = hp_num
filter_coeffs[2][:] = lp_denom
filter_coeffs[3][:] = lp_num
        
filter_ICs[1,:] = sig.lfilter_zi(filter_coeffs[3][:],filter_coeffs[2][:])
filter_ICs[0,:] = sig.lfilter_zi(filter_coeffs[1][:],filter_coeffs[0][:])

filter_ICs_IR[1,:] = sig.lfilter_zi(filter_coeffs[3][:],filter_coeffs[2][:])
filter_ICs_IR[0,:] = sig.lfilter_zi(filter_coeffs[1][:],filter_coeffs[0][:])
# ==================== Globals ====================
N = 400                                       # samples to plot
NS = 20                                         # samples to grab each iteration
sample_count = 0                                # current sample count
times, values, values2, values3, values4, values5, values6, values7 = np.zeros((8, N))       # data vectors
serial_port = 'COM4' # 'COM3'   # the serial port to use
serial_baud = 9600   
count = 0                         

#============================READ BLE=============================
def read_BLE( ser ):
	msg = ""
	if( ser.in_waiting > 0 ):
    		msg = ser.readline( ser.in_waiting ).strip().decode('utf-8')
	return msg

# ==================== Grab 'count' samples from Serial ====================
def grab_samples( n_samples ):
    global sample_count # using global variable 'sample_count'
    t, a, b, c, d, e, f, log, proc_data = np.zeros( (9, n_samples) )
    i = 0
    while (i < n_samples) :
        
        data = ""
        try:
#                data = read_BLE( ser )
            data = ser.readline().decode('utf-8').strip()
            ti, gz, ax, az, BPM = data.split(' ')
            t[i] = float(ti)/1000000.0
            a[i] = float(gz)
            b[i] = float(ax)
            c[i] = float(az)
            
#            print('Heart Rate: ')
#            print(BPM)
#            print('\n')
            
        except ValueError:
            print('Invalid data: ', data)
            while (data == "1") :
                command = 'AT+IMME1'
                ser.write(command.encode('utf-8'))
                sleep(2)
                command = 'AT+ROLE1'
                ser.write(command.encode('utf-8'))
                sleep(2)
                command = 'AT+RESET'
                ser.write(command.encode('utf-8'))
                sleep(2)
                command = 'AT+CON3403DE02BAC3'
                ser.write(command.encode('utf-8'))
                sleep(2)
                print(command)
                sleep(2)
                data = ser.readline().decode('utf-8').strip()
                
            continue 
        i += 1

    sample_count += n_samples
    return t, a, b, c, d

# ==================== Grab new samples and plot ====================
def update_plots(i):
    global times, values, hr_data, filter_ICs, filter_ICs_IR, proc_data, location

    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    values[:N-NS] = values[NS:]
    proc_data[:N-NS] = proc_data[NS:]
    values2[:N-NS] = values2[NS:]
    values3[:N-NS] = values3[NS:]
#    hr_data[:N-NS] = hr_data[NS:]
#    location[:N-NS] = location[NS:]


    # grab new samples
    times[N-NS:], values[N-NS:], values2[N-NS:], values3[N-NS:], BPM = grab_samples(NS)
    
    proc_data[N-NS:], filter_ICs = filtering.process_ir(values[N-NS:],filter_coeffs, filter_ICs, 0)
#    hr_data[N-NS:], filter_ICs_IR = filtering.process_ir(values2[N-NS:],filter_coeffs, filter_ICs_IR, 0)
    
    
#    y = np.zeros((2, N)) 
#    y[0][:] = times
#    y[1][:] = values2
#    hr, location = calculate_hr(y)
    
    threshold = 3000
    if values[350] > threshold:
            hello =  '1'
            ser.write(hello.encode('utf-8'))     
            
    elif values[350] < -(threshold):
            hello = '1'
            ser.write(hello.encode('utf-8'))
    
    if values2[350] > 8000 and values3[350] < -8000: 
            hello = '2'
            ser.write(hello.encode('utf-8'))
            print('SENT A 2')

    
    # plot
    [ax.set_xlim(times[0],times[N-1]) for ax in axes]
    live_plots[0].set_data(times, values)
    live_plots[1].set_data(times,proc_data)
    live_plots[2].set_data(times, values2)
    live_plots[3].set_data(times, values3)

# ==================== Main ====================
if (__name__ == "__main__") :
    with serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1) as ser:
        # initialize the figure 
        #INITIAL BLE CONNECTION
        sleep(3)
        command = 'AT+IMME1'
        ser.write(command.encode('utf-8'))
        sleep(3)
        print('got ' + ser.readline().decode('utf-8').strip())
        command = 'AT+ROLE1'
        ser.write(command.encode('utf-8'))
        sleep(3)
        print('got ' + ser.readline().decode('utf-8').strip())
        command = 'AT+RESET'
        ser.write(command.encode('utf-8'))
        sleep(8)
        print('got ' + ser.readline().decode('utf-8').strip())
        
        command = 'AT+CON3403DE02BAC3'
        ser.write(command.encode('utf-8'))
        sleep(5)
        print('got ' + ser.readline().decode('utf-8').strip())
        resp = ser.readline().decode('utf-8').strip()
        while('OK+CONNA' in resp):
            ser.write(command.encode('utf-8'))
            sleep(5)
            resp = ser.readline().decode('utf-8').strip()
            print('got ' + resp)
            
        print("BLE Connected")
        sleep(5)
                
    #CONNECTING ADDRESS
    #AT+CON3403DE1AA4B3
    
    #WAKE COMMAND
    #AT+AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
        
         
        fig, axes = plt.subplots(4, 1)  

        times, gz, ax, az, BPM = grab_samples(N)
        proc_data, filter_ICs = filtering.process_ir(ax,filter_coeffs,filter_ICs, 1)
#        hr_data, filter_ICs_IR = filtering.process_ir(heartrate,filter_coeffs,filter_ICs_IR, 1)
#        y = np.zeros((2, N)) 
#        y[0][:] = times
#        y[1][:] = values
#        hr, location = calculate_hr(y)
        
        
        live_plots = []
        live_plots.append(axes[0].plot(times, gz, lw=2)[0])
        live_plots.append(axes[1].plot(times, proc_data, lw=2)[0])
        live_plots.append(axes[2].plot(times, ax, lw=2)[0])
        live_plots.append(axes[3].plot(times, az, lw=2)[0])
        
        # initialize the y-axis limits and labels
        axes[0].set_ylim(-20000, 20000)
        axes[1].set_ylim(-20000, 20000)
        axes[2].set_ylim(-20000, 20000)
        axes[3].set_ylim(-20000, 20000)
    
        axes[0].set_title('GZ')
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Amplitude')
        
        axes[1].set_title('Filtered GZ')
        axes[1].set_ylabel('Amplitude')
        axes[1].set_xlabel('Time (s)')
        
        axes[2].set_title('AX')
        axes[2].set_ylabel('Amplitude')
        axes[2].set_xlabel('Time (s)')
        
        axes[3].set_title('AZ')
        axes[3].set_ylabel('Amplitude')
        axes[3].set_xlabel('Time (s)')
    
        # set and start the animation and update at 1ms interval (if possible)
        anim = animation.FuncAnimation(fig, update_plots, interval=1)
        plt.show()
