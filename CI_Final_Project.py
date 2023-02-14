'''
CI_Final_Project:

Acceralation Adjusting Using fuzzy logic
Here we want to keep the distance between two cars close to a certain value (d)
Fuzzy system has two inputs (Error and delta), which error is deviation of d and delta is the first derivative of error, since error is changing.
in order to run this code, you need to install these modules:

```
pip install numpy
pip install -U scikit-fuzzy
pip install -U matplotlib
```

'''

import math
import random
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl


def gastest():

    # Input acceleration & velocity & distance for two Cars
    print("Please input the First car's Acceleration & Velocity (Car1: 0.5 * a(t)^2 + v0(t) + x0)\n& the distance you want these cars have\n ")
    init_a = float(input("First Acceleration(Should be between -10 to 10): "))
    init_v0 = float(input("First Velocity(Should be between -30 to 30): "))
    init_d = float(input("Distance Between Cars(Should be between 10 to 100): "))
    init_x0 = 55

    feedback(init_a, init_v0, init_d, init_x0)


def randomSafetyTest(times):
    # Randomly produce initial variables: acceleration & velocity & distance for number of times user inputs
    for test in range(times):

        init_a = float(random.randint(-10,10))
        init_v0 = float(random.randint(-30,30))
        init_d = float(random.randint(10,100))
        init_x0 = 55
        
        feedback(init_a, init_v0, init_d, init_x0)
    
def feedback(firstCarA, firstCarV, distance, firstCarX0):
    # Entire processing simulation

    # Time unit 
    time = 1/8

    # For the first time Second Car Speed is equal to First Car Speed
    secondCarV = firstCarV

    # First distance between two cars
    lastDistance = 55

    # List for distance & velocity & gas to plot them
    list_d = []
    list_v = []
    list_g = []

    # Our Entire Time scope is 25s 
    for i in range(0,200):
        
        # Calculate the distance betwween two cars
        firstCarX = firstCarA * ((time*i) ** 2) / 2 + firstCarV * (time*i) + firstCarX0
        secondCarX = secondCarV * (time*i) 
        carsDistance = round((firstCarX - secondCarX),2)

        # Calculate the error between the Cardistance & Our expected distance
        error = distance - carsDistance
        delta_error = lastDistance - carsDistance
        lastDistance = carsDistance

        # Pass error & delta_error to fuzzy control system
        gas = FLctrl(error, delta_error)

        # Calculate the next round speed
        secondCarV += gas * time
        secondCarV, gas = round(secondCarV,2), round(gas,2)

        # Append everything to lists
        list_d.append(carsDistance)
        list_v.append(secondCarV)
        list_g.append(gas)

        # Print the pressure we must put on Second Car's gas pedal
        print('Speed is: {0:.2f}m/s, Carsdistance is: {1:.2f}m, Gas force is: {2:.2f}kN'.format(secondCarV, carsDistance, gas))

    # Print the initional varieable 
    print("================================================================================================")
    print (firstCarA, firstCarV, distance, firstCarX0)
    print("================================================================================================")

    # Plot the result
    number =  np.arange(0.0, 25.0, 0.125)
    plt.plot(number, list_v, color='r', label='Vlocity Car2')
    plt.plot(number, list_d, color='b', label='CarsDistance')
    plt.plot(number, list_g, color='g', label='Gas')

    plt.xlabel("Time")
    plt.ylabel("")
    plt.title("Fuzzy Control System")

    plt.legend()
    plt.show()

def FLctrl(current_error, current_delta_error):
    
    # Define input and output variables
    error = ctrl.Antecedent(np.arange(-50, 50, 1), 'error')
    delta_error = ctrl.Antecedent(np.arange(-10, 10, 1), 'delta_error')
    gas = ctrl.Consequent(np.arange(-24, 24, 1), 'gas')

    # error membership function
    error['low'] = fuzz.trimf(error.universe, [-50, -50, -25])
    error['mediumLow'] = fuzz.trimf(error.universe, [-50, -25, 0])
    error['medium'] = fuzz.trimf(error.universe, [-25, 0, 25])
    error['mediumHigh'] = fuzz.trimf(error.universe, [0, 25, 50])
    error['high'] = fuzz.trimf(error.universe, [25, 50, 50])

    # delta_error membership function
    delta_error['low'] = fuzz.trimf(delta_error.universe, [-10, -10, -5])
    delta_error['mediumLow'] = fuzz.trimf(delta_error.universe, [-10, -5, 0])
    delta_error['medium'] = fuzz.trimf(delta_error.universe, [-5, 0, 5])
    delta_error['mediumHigh'] = fuzz.trimf(delta_error.universe, [0, 5, 10])
    delta_error['high'] = fuzz.trimf(delta_error.universe, [5, 10, 10])

    # gas force membership function
    gas['low'] = fuzz.trimf(gas.universe, [-24, -24, -12])
    gas['mediumLow'] = fuzz.trimf(gas.universe, [-24, -12, 0])
    gas['medium'] = fuzz.trimf(gas.universe, [-12, 0, 12])
    gas['mediumHigh'] = fuzz.trimf(gas.universe, [0, 12, 24])
    gas['high'] = fuzz.trimf(gas.universe, [12, 24, 24])


    # Inference Engine
    rule1 = ctrl.Rule(antecedent=(error['mediumHigh'] & delta_error['high']) |
                                 (error['high'] & delta_error['mediumHigh']) |
                                 (error['high'] & delta_error['high']),
                             consequent=gas['low'])

    rule2 = ctrl.Rule(antecedent=(error['mediumLow'] & delta_error['high']) |
                                 (error['medium'] & delta_error['high']) |
                                 (error['mediumHigh'] & delta_error['mediumHigh']) |
                                 (error['mediumHigh'] & delta_error['medium']) |
                                 (error['high'] & delta_error['medium']) |
                                 (error['high'] & delta_error['mediumLow']) |
                                 error['high'] & delta_error['low'],
                             consequent=gas['mediumLow'])

    rule3 = ctrl.Rule(antecedent=(error['medium'] & delta_error['mediumLow']) |
                                 (error['mediumLow'] & delta_error['mediumHigh']) |
                                 (error['medium'] & delta_error['medium']) |
                                 (error['medium'] & delta_error['mediumHigh']) |
                                 (error['mediumHigh'] & delta_error['mediumLow']),
                             consequent=gas['medium'])

    rule4 = ctrl.Rule(antecedent=(error['low'] & delta_error['high']) |
                                 (error['low'] & delta_error['medium']) |
                                 (error['low'] & delta_error['mediumHigh']) |
                                 (error['mediumLow'] & delta_error['mediumLow']) |
                                 (error['mediumLow'] & delta_error['medium']) |
                                 (error['medium'] & delta_error['low']) |
                                 (error['mediumHigh'] & delta_error['low']),
                             consequent=gas['mediumHigh'])

    rule5 = ctrl.Rule(antecedent=(error['low'] & delta_error['low']) |
                                 (error['low'] & delta_error['mediumLow']) |
                                 (error['mediumLow'] & delta_error['low']),
                             consequent=gas['high'])

    # Controller simulation
    gas_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
    gas = ctrl.ControlSystemSimulation(gas_ctrl)

    # Automatic gas processing
    gas.input['error'] = current_error
    gas.input['delta_error'] = current_delta_error
    gas.compute()
    # gas.view(sim = gas)
    return gas.output['gas']


if __name__ == "__main__":
    mode = input("Please select mode:\n\
                  0: Single Gas Test\n\
                  1: Random Safety Test and Regression Analysis\n")
    if mode == '0':
        gastest()
    elif mode == '1':
        tests = int(input("Please input your test times you want to run the code:"))
        randomSafetyTest(tests)
    else:
        print("Wrong mode type! Please try again!")

    
