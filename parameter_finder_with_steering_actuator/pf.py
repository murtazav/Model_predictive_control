# best controller parameterfinder using genetic algorithm
import numpy as np
import random
from matplotlib import pyplot as plt
population = 20
kp_arr = np.zeros(population)
ki_arr = np.zeros(population)
fit_arr = np.zeros(population)
kp_max = 100.0
kp_min = 0.0
ki_max = 100.0
ki_min = 0.0
itr = 50


#environment with bicycle model which returns fitness of a controller
def envm(kp, ki):
    mfl=158 # mass at front left wheel
    mfr=137 # mass at front right wheel
    mrl=360 # mass at rear left wheel
    mrr=269 # mass at front right wh
    m = 924 #mass of the vehicle
    lf= 1.31 # location of centre of gravity from the front axle
    lr=0.62 # location of centre of gravity from the rear axle
    Iz=748 # moment of inertia
    l=lf+lr
    V=1
    kd = 0
    #parametres of steering actuator circuit
    R = 0.317
    L = 0.0000823
    kt = 0.0302
    kb = 0.0301
    b = 0.0028677
    
    #cornering stiffness of tyre
    Cf=27359
    Cr=58535
    dt=0.001
    tf=10
    vy = np.zeros(15000)
    vy_dot = np.zeros(15000)
    delt = np.zeros(15000)
    theta_dot = np.zeros(15000)
    theta = np.zeros(15000)
    theta_des = np.zeros(15000)
    theta_ddot = np.zeros(15000)
    e = np.zeros(15000)
    e_int = np.zeros(15000)
    e_d = np.zeros(15000)
    e_phi=np.zeros(15000)
    eint_phi = np.zeros(15000)
    phi = np.zeros(15000)
    phi_des = np.zeros(15000)
    phi_dot = np.zeros(15000)
    kp_phi = 2
    ki_phi = 0
    v = np.zeros(15000)
    I = np.zeros(15000)
    fitness =0
    time = np.zeros(15000)
    for i in range(1,len(time)):
        time[i] = time[i-1]+dt
    i=1
    for t in time[0:10000]:
        if i>1000:
            theta_des[i] = 0.349
        else:
            theta_des[i] = 0
        e[i] = theta_des[i]-theta[i-1]
        fitness = fitness - np.power(t,2)*np.absolute(e[i])*dt
        e_int[i] = e_int[i-1]+e[i]*0.001
        if i>2:
            e_d[i] = (theta[i-1]-theta[i-2])/dt
        phi_des[i] = kp*e[i]+ki*e_int[i]+kd*e_d[i] 
        e_phi[i] = phi_des[i]-phi[i-1]
        eint_phi[i]=eint_phi[i-1]+dt*e_phi[i]
        v[i]=3555*(kp_phi*e_phi[i]+ki_phi*eint_phi[i])
        if v[i]>0:
            v[i]=20
        elif v[i]<0:
            v[i]=-20
        I[i]=L/R*(v[i]-kb*phi_dot[i-1])-L/(R*np.exp(R*t/L))*(v[i]-kb*phi_dot[i-1])
        phi_dot[i]=kt/b*I[i]
        phi[i]=phi[i-1]+dt*phi_dot[i]
        delt[i]=1/3555*phi[i]*6863.6364
           	#to use arc tan function from numpy library
        vy_dot[i]=-V*theta_dot[i-1]+1/m*(Cr*(-np.arctan((vy[i-1]-lr*theta_dot[i-1])/V))+Cf*(delt[i]-np.arctan((vy[i-1]+lf*theta_dot[i-1])/V))*np.cos(delt[i]))
        theta_ddot[i]=1/Iz*(Cr*lr*(-np.arctan((vy[i-1]-lr*theta_dot[i-1])/V))+Cf*(delt[i]-np.arctan((vy[i-1]+lf*theta_dot[i-1])/V))*np.cos(delt[i])*lf)
        theta_dot[i]=theta_ddot[i]*dt +theta_dot[i-1]
        theta[i]=theta_dot[i]*dt+theta[i-1]
        vy[i]=vy_dot[i]*dt+vy[i-1]
        i+=1
    #print(fitness)
    plt.plot(time[0:10000],theta[0:10000]*180/np.pi)
    plt.plot(time[0:10000],theta_des[0:10000]*180/np.pi)
    plt.plot(time[0:10000],delt[0:10000]*180/np.pi)
    plt.legend(['heading_angle','desired heading','steering angle'])
    plt.xlabel('time in s')
    plt.ylabel('Angle in degrees')
    # linearly penalizing max steering angle, max overshoot, randomly improving fitness with mean 5 and std 5 to introduce luck in evolution process
    fitness = 10*fitness - 50*np.amax(delt[0:10000]) -100*(np.amax(theta[0:10000])-np.amax(theta_des[0:10000])) + (np.random.randn()*5+5)
    return fitness


# to randomly generate a contorller within the range
def random_birth(kp_arr, ki_arr):
    for i in range(population):
        kp_arr[i] = random.uniform(kp_min, kp_max)
        ki_arr[i]= random.uniform(ki_max, kp_min)
    return kp_arr, ki_arr
# to generate offspring from the best parents with added mutation
def offspring(kp_mean, kp_std, ki_mean, ki_std):
    kp = np.absolute(np.random.randn()*kp_mean +kp_std)
    ki = np.absolute(np.random.randn()*ki_mean + ki_std)
    #mutation
    if random.random() < 0.1 :
        kp = np.absolute(kp + 0.1*(np.random.randn()*kp_mean+kp_std))
        ki = np.absolute(ki + 0.1*(np.random.randn()*ki_mean+ki_std))
    return kp, ki


# to iteratively create and delete generations to reach the generaton with best fitness score
def iterations(kp_arr, ki_arr, fit_arr):
    for i in range(itr):
        for j in range(population):
            fit_arr[j] = envm(kp_arr[j], ki_arr[j])
        # sort remove the half controllers with worst fitness
        sorted_index = np.argsort(fit_arr)
        kp_arr = kp_arr[sorted_index]
        ki_arr = ki_arr[sorted_index]
        fit_arr = fit_arr[sorted_index]
        print(str(kp_arr[-1])+' '+str(ki_arr[-1])+' '+str(fit_arr[-1]))
        # generate offsprings with the best four controllers
        kp_mean = np.average(kp_arr[int(population/2):population])
        kp_std = np.std(kp_arr[int(population/2):population])
        ki_mean = np.average(ki_arr[int(population/2):population])
        ki_std = np.std(ki_arr[int(population/2):population])
        for j in range(int(population/2)):
            kp_arr[j], ki_arr[j] = offspring(kp_mean, kp_std, ki_mean, ki_std)
    return kp_arr, ki_arr, fit_arr


kp_arr, ki_arr = random_birth(kp_arr, ki_arr)
iterations(kp_arr, ki_arr, fit_arr)
