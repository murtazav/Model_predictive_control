1. Setpoint / Reference following:
Parameters to maximize or minimize for setpoint following depends on the required performance. In general, the parameters are rise time, settling time, decay ratio, overshoot and steady state error.

2. To characterize the performance of reference following the criterion used are:
(Reference:  PID Controllers: Theory, design and tuning by Karl J. Astrom and Tore Hagglund)
IAE:  Integrated absolute error                                                       ∫t^2 |e(t)|dt
ITE: Integrated time error                                                            ∫t^2 dt
ITSE: Integrated time squared error                                                   ∫t*e(t)^2 dt
ISTE: Integrated squared time error.                                                  ∫t^2*e(t)dt
ISTAE: Integrated squared time absolute error                                         ∫t^2*|e(t)|dt 


3. Parameters and criterion used for measuring fitness of a controller:
ISTAE, overshoot, maximum steering angle 

4. Genetic algorithm:
	https://towardsdatascience.com/introduction-to-genetic-algorithms-including-example-code-e396e98d8bf3
	 http://blog.otoro.net/2017/10/29/visual-evolution-strategies/
	D. C. Meena, Ambrish Devanshu, Genetic Algorithm Tuned PID Controller for Process Control, 
  International Conference on Inventive Systems and Control (ICISC-2017)


5. Steps:
	Choosing the population size
	Maximum and minimum of attributes (kp, ki)
	No. of generations to iterate to get generation with best fitness.
	Defining the environment that decides the fitness of a controller: chosen fitness function:
-10*∫t^2 |e(t)|dt-50*max(steeringangle)-100*overshoot+randomluck[-5,5] 
	Before iterations begin a random set of controller population is generated.
In every iteration 
	The fitness of all the members of the population is calculated.
	The population is sorted based on the fitness of individuals.
	The worst half of the population of controllers is deleted.
	By using the mean and standard deviation of the attributes of the best half controllers equal
  amount of off springs are generated with added mutation.
	The loop is repeated again until a given amount of generations.
	The attributes of the best controllers of the latest generation is evaluated graphically. 

6. Implementation on the 2012 model:
The vehicle dynamics model plus the genetic algorithm was implemented in python using ‘numpy’,
‘matplotlib’ and ‘random’ libraries and the code was executed on google colaboratory.research (online python notebook). 
Performance results of best three controllers of the 50th generation:
 
7. Implementation on the 2015 model:
bicycle model with steering actuator.
Performance results of best controller of the 50th generation:

8. Conclusion: We can observe that the best performing controllers (with highest fitness) 
of the last generation are able to follow the desired heading angle (for both with and without
steering actuator) with good characteristics (such as low-rise time, low settling time, low 
overshoot and maximum steering angle within its limit). With every generation there is an 
improvement in average fitness (i.e., performance). But higher fitness doesn’t necessarily 
mean better performance i.e., controllers with almost similar fitness may show variation in 
performance (not huge difference). This is due to the luck factor included in the fitness 
calculation of controllers and also due to added mutation in the produced offspring controllers
in every cycle.

