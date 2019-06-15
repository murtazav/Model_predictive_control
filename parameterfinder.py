# best controller parameterfinder using genetic algorithm
import numpy as np
import random
from matplotlib import pylot as plt

cntrlrs = {}
for i in range(8):
	cntrlrs['c'+str(i)]=[NaN,NaN,NaN]
kp_max = 100.0
kp_min = 0.0
ki_max = 100.0
ki_min = 0.0
itr = 1000

#environment with bicycle model which returns fitness of a controller
def envm(kp, ki):
	mfl=158  # mass at front left wheel
	mfr=137  # mass at front right wheel
	mrl=360  # mass at rear left wheel
	mrr=269  # mass at front right wh

	m = 924  #mass of the vehicle

	lf= 1.31  # location of centre of gravity from the front axle
	lr=0.62   # location of centre of gravity from the rear axle
	Iz=748    # moment of inertia
	l=lf+lr

	V=1
	#cornering stiffness of tyre
	Cf=27359
	Cr=58535
	dt=0.001
	tf=80
	vy = np.zeros(100000)
	delt_4 = np.zeros(100000) 
	theta_dot = np.zeros(100000)
	theta_4 = np.zeros(100000)
	theta_des = np.zeros(100000)
	theta_ddot = np.zeros(100000)
	e = np.zeros(100000)
	e_int = np.zeros(100000)
	fitness =0
	time = np.zeros(100000)
	for i in range(1,len(time)):
		time[i] = time[i-1]+dt
	i=1
	for t in time[0:80000]:
		if i>=5000:
		theta_des(i)=0.349

		e[i] = (theta_des[i]-theta_4[i-1])
		e_int[i] = e_int[i-1]+e[i]*0.001
		delt_4[i]=kp*e[i]+ki*e_int[i]

		vy_dot[i]=-V*theta_dot[i-1]+1/m*(Cr*(-np.arctan((vy[i-1]-lr*theta_dot[i-1])/V))+Cf*(delt_4[i]-np.arctan((vy[i-1]+lf*theta_dot[i-1])/V))*np.cos(delt_4[i]))
		theta_ddot[i]=1/Iz*(Cr*lr*(-np.arctan((vy[i-1]-lr*theta_dot[i-1])/V))+Cf*(delt_4[i]-np.arctan((vy[i-1]+lf*theta_dot[i-1])/V))*cos(delt_4[i])*lf)

		theta_dot[i] = theta_ddot[i]*dt +theta_dot[i-1]
		theta_4[i] = theta_dot[i]*dt+theta_4[i-1]
		vy([i] = vy_dot[i]*dt+vy[i-1]
		i=i+1
		fitness -= t*t*abs(e[i]) 
	plt.plot(time{0:80000],theta_4[0:80000)
	return fitness

# to randomly generate a contorller within the range
def random_birth():
	for i in cntrlrs.keys():
		kp = random.uniform(kp_min, kp_max)
		ki = random.uniform(ki_max, kp_min)
		i[0], i[1] = kp, ki

# to generate offspring from the best parents with added mutation
def offspring(kp_mean, kp_std, ki_mean, ki_std):
	kp = random.random()*kp_mean +kp_std
	ki = random.random()*ki_mean + ki_std
	
	#mutation
	# if random.random < 0.1 :
		# kp = kp + random.choice([1,-1])*0.01*(random.random()*kp_mean+kp_std)
		# ki = ki + random.choice([1,-1])*0.01*(random.random()*ki_mean+ki_std)
	return kp, ki
	

# to iteratively create and delete generations to reach the generaton with best fitness score
def iterations():
	for i in itr:
		for j in cntrlrs.keys():
			j[2] = envm(j[0],j[1])
		
		# sort remove the 4 controllers with worst fitness
		cntrlrs = sorted(cntrlrs.values(), key = lambda k: cntrlrs[k][3], reverse = True)
		# generate offsprings with the best four controllers
		kp_array = np.zeros(4)
	ki_array = np.zeros(4)
	for i in range(4):
		kp_array[i] = cntrlrs['c'+str(i)][0]
		ki_array[i] = cntrlrs['c'+str(i)][1]
	kp_mean = np.mean(kp_array)
	kp_std = np.std(kp_array)
	ki_mean = np.mean(ki_array)
	ki_std = np.std(ki_array)
		for j in range(4:8):
			cntrls['c'+str(i)][0], cntrls['c'+str(i)][1] = offspring(kp_mean, kp_std, ki_mean, ki_std)
	print(cntrlrs['c0'][2])	
	
random_birth()
iterations()