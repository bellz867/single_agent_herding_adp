#!/usr/bin/env python

import matplotlib.pyplot as plt
import csv
import numpy as np
import math

time = []
eta0 = []
eta1 = []
etad0 = []
etad1 = []
uhat0 = []
uhat1 = []
etaDot0 = []
etaDot1 = []
wc0 = []
wc1 = []
wc2 = []
wc3 = []
wc4 = []
wc5 = []
wc6 = []
wa0 = []
wa1 = []
wa2 = []
wa3 = []
wa4 = []
wa5 = []
wa6 = []
uwall0 = []
uwall1 = []
mud0 = []
mud1 = []
muhat0 = []
muhat1 = []
x0 = []
x1 = []
x2 = []
x3 = []
x4 = []
x5 = []

timesheep = []
z0 = []
z1 = []

kd = 1.15
zg0 = -2.0
zg1 = 0.0

exp = 'expx8/'

#"time," << "z0," << "z1," << "zDot0," << "zDot1," << "chased," << "thHat00," << "thHat10,"
              #<< "thHat20," << "thHat30," << "thHat01," << "thHat11," << "thHat21," << "thHat31,"
              #<< "Sz0," << "Sz0," << "Sz1," << "Sz2," << "Sz3," << "\n";
with open(exp+'sheep1.txt','r') as csvfile:
	data = csv.reader(csvfile,delimiter=',')
	next(data)
	for row in data:
		timesheep.append(float(row[0]))
		z0.append(float(row[1]))
		z1.append(float(row[2]))


#time,eta0,eta1,etaDot0,etaDot1,uHat0,uHat1,WcHat0,WcHat1,WcHat2,
#WcHat3,WcHat4,WcHat5,WcHat6,WaHat0,WaHat1,WaHat2,WaHat3,WaHat4,WaHat5,
#WaHat6,uWall0,uWall1,mud0,mud1,muhat0,muhat1


with open(exp+'bebop4.txt','r') as csvfile:
	data = csv.reader(csvfile,delimiter=',')
	next(data)
	for row in data:
		time.append(float(row[0]))
		eta0.append(float(row[1]))
		eta1.append(float(row[2]))
		etaDot0.append(float(row[3]))
		etaDot1.append(float(row[4]))
		uhat0.append(float(row[5]))
		uhat1.append(float(row[6]))
		wc0.append(float(row[7]))
		wc1.append(float(row[8]))
		wc2.append(float(row[9]))
		wc3.append(float(row[10]))
		wc4.append(float(row[11]))
		wc5.append(float(row[12]))
		wc6.append(float(row[13]))
		wa0.append(float(row[14]))
		wa1.append(float(row[15]))
		wa2.append(float(row[16]))
		wa3.append(float(row[17]))
		wa4.append(float(row[18]))
		wa5.append(float(row[19]))
		wa6.append(float(row[20]))
		
		uwall0.append(float(row[21]))
		uwall1.append(float(row[22]))
		mud0.append(float(row[23]))
		mud1.append(float(row[24]))
		muhat0.append(float(row[25]))
		muhat1.append(float(row[26]))
		x0.append(float(row[27]))
		x1.append(float(row[28]))
		x2.append(float(row[29]))
		x3.append(float(row[30]))
		x4.append(float(row[31]))
		x5.append(float(row[32]))
		etad0.append(float(row[29])+kd*float(row[27])+zg0)
		etad1.append(float(row[30])+kd*float(row[28])+zg1)

wcplt,wcax = plt.subplots()
wcax.plot(time,wc0,label='wc0')
wcax.plot(time,wc1,label='wc1')
wcax.plot(time,wc2,label='wc2')
wcax.plot(time,wc3,label='wc3')
wcax.plot(time,wc4,label='wc4')
wcax.plot(time,wc5,label='wc5')
wcax.plot(time,wc6,label='wc6')
wcax.set_xlabel('time')
wcax.set_ylabel('weight')
wcax.set_title('critic')
wcax.legend()
wcax.grid()
wcplt.show()

waplt,waax = plt.subplots()
waax.plot(time,wa0,label='wa0')
waax.plot(time,wa1,label='wa1')
waax.plot(time,wa2,label='wa2')
waax.plot(time,wa3,label='wa3')
waax.plot(time,wa4,label='wa4')
waax.plot(time,wa5,label='wa5')
waax.plot(time,wa6,label='wa6')
waax.set_xlabel('time')
waax.set_ylabel('weight')
waax.set_title('actor')
waax.legend()
waax.grid()
waplt.show()

cmdplt,(cmdax0,cmdax1) = plt.subplots(nrows=1,ncols=2)
#cmdax0.plot(time,uwall0,label='uwall0')
#cmdax1.plot(time,uwall1,label='uwall1')
cmdax0.plot(time,mud0,label='mud0')
cmdax1.plot(time,mud1,label='mud1')
cmdax0.plot(time,muhat0,label='muhat0')
cmdax1.plot(time,muhat1,label='muhat1')
cmdax0.plot(time,uhat0,label='uhat0')
cmdax1.plot(time,uhat1,label='uhat1')
cmdax0.plot(time,etaDot0,label='etaDot0')
cmdax1.plot(time,etaDot1,label='etaDot1')
cmdax0.set_xlabel('time')
cmdax1.set_xlabel('time')
cmdax0.set_ylabel('cmd')
cmdax1.set_ylabel('cmd')
cmdax0.set_title('x command')
cmdax1.set_title('y command')
cmdax0.legend()
cmdax1.legend()
cmdax0.grid()
cmdax1.grid()
cmdplt.show()

xplt,(xax0,xax1) = plt.subplots(nrows=1,ncols=2)
xax0.plot(time,x0,label='ez0')
xax1.plot(time,x1,label='ez1')
xax0.plot(time,x2,label='ed0')
xax1.plot(time,x3,label='ed1')
xax0.plot(time,x4,label='eeta0')
xax1.plot(time,x5,label='eeta1')
xax0.set_xlabel('time')
xax1.set_xlabel('time')
xax0.set_ylabel('error')
xax1.set_ylabel('error')
xax0.set_title('x error')
xax1.set_title('y error')
xax0.legend()
xax1.legend()
xax0.grid()
xax1.grid()
xplt.show()

theta = np.arange(-2*math.pi,2.0*math.pi,0.01)
sinth = []
costh = []
r = 0.65
cx = -2.0
cy = 0.0
for thetai in theta:
	sinth.append(cy + r*math.sin(thetai))
	costh.append(cx + r*math.cos(thetai))

etazplt,etazax = plt.subplots()
etazax.plot(costh,sinth,color='red')
etazax.plot(z0[0],z1[0],label='z0',color='green',marker='o',markersize=15,fillstyle='none')
etazax.plot(z0,z1,color='green',label='z')
etazax.plot(z0[len(z0)-1],z1[len(z1)-1],color='green',marker='x',markersize=15,label='zf')
etazax.plot(eta0[0],eta1[0],label='eta0',color='blue',marker='o',markersize=15,fillstyle='none')
etazax.plot(eta0,eta1,color='blue',label='eta')
etazax.plot(eta0[len(eta0)-1],eta1[len(eta1)-1],label='etaf',color='blue',marker='x',markersize=15)
etazax.plot(etad0[0],etad1[0],label='etad0',color='cyan',marker='o',markersize=15,fillstyle='none')
etazax.plot(etad0,etad1,color='cyan',label='etad')
etazax.plot(etad0[len(etad0)-1],etad1[len(etad1)-1],label='etadf',color='cyan',marker='x',markersize=15)
etazax.set_xlabel('x (m)')
etazax.set_ylabel('y (m)')
etazax.legend()
etazax.grid()
etazax.axis('equal')
etazplt.show()

try:
	input("use any key to exit")
except SyntaxError:
	pass



