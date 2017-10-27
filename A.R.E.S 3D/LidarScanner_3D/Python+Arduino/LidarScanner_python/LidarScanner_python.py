import serial
from vpython import *
import re

inputdata = serial.Serial('/dev/ttyACM2', 115200)

# points(pos=[(0,0,0)],radius=1,color=color.red)

ball = sphere(pos=vector(0,0,0),radius=1,make_trail=True,trail_type="points",retain=1000000,trail_radius=1,interval=1)

# ball = sphere(pos=vector(0,0,0),radius=1,make_trail=True,trail_type="curve",retain=1000000,trail_radius=0)


i=0
j=0
k=0

while not False:
	if (inputdata.inWaiting() > 0):
		pointData = inputdata.readline()
		# pointData = str(i)+str(' ')+str(j)+str(' ')+str(k)
		# i=i+1
		# j=j+1
		# k=k+1
		print(pointData)

		try:
			pointData = [float(l) for l in pointData.split()]
		except:
			print("First Except")
			continue

		# pointData = map(int,pointData.split())
		# pointDatav = []
		# for item in pointData.split():
			# pointDatav.append(item)

		print(pointData)
		try:
			# points(pos=vector(pointData[0],pointData[1],pointData[2]),radius=1,color=color.white)
			ball.pos=vector(pointData[0],pointData[1],pointData[2])
			print("plottted"+str(pointData))
		except:
			print("Second Except")
			continue

		# if (k == 200):
			# quit()