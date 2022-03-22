# -*- coding: utf-8 -*-b
"""
Created on Tue Apr  9 18:24:48 2019
@author: User1
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime
import serial
import csv

fig, ax = plt.subplots()
xdat,xdata, ydata,zdata = list(range(1,501)), [0]*500, [0]*500, [0]*500
line1, = ax.plot(xdat,xdata)
line2, = ax.plot(xdat,ydata)
line3, = ax.plot(xdat,zdata)
ax.set_ylim(-8, 8)   #設置圖形的y軸上下限

COM_PORT = 'COM9'    # 指定通訊埠名稱
BAUD_RATES = 115200  # 設定傳輸速率
#BAUD_RATES = 1000000  # 設定傳輸速率
ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠

row,totalrow = [],[]
i = 0
time = 1

def s16(value):
    value = int(value,16)
    return -(value & 0x8000) | (value & 0x7fff)

def s8(value):
    value = int(value,8)
    return -(value & 0x80) | (value & 0x7f)

def update(data):

    line1.set_ydata(xdata)
    line2.set_ydata(ydata)
    line3.set_ydata(zdata)
    return line1,line2,line3 

def data_gen(): 
    global time
    try:
        while ser.in_waiting:          # 若收到序列資料…
            start = datetime.datetime.now()
            data_raw = ser.readline()
            data = data_raw.decode()
            #data = data.split(',')
            for loop in range(0,10):
                x = s16(data[loop*12+0:loop*12+4])/256
                y = s16(data[loop*12+4:loop*12+8])/256
                z = s16(data[loop*12+8:loop*12+12])/256
                del xdata[0]    #刪除list最左邊的項,並且整個list的數據會向左移動,故剩下最右項沒有值
                del ydata[0]
                del zdata[0]
                xdata.insert(500,x)    #加入uart新傳入的值到最右項中
                ydata.insert(500,y)
                zdata.insert(500,z)
                if loop==0:
                    num = time
                else:
                    num = 0
            #print (datetime.datetime.now()- start)
            #print(data_raw)
            #print(x,y,z)
            time=time+1
            
    except:
        print(data_raw)
        print("signal stop")
    yield 0

ani = animation.FuncAnimation(fig, update, data_gen, interval=30, blit=True)
plt.legend(handles = [line1,line2,line3],labels=['acc_x','acc_y','acc_z'],loc='upper left')
plt.show() #顯示圖形