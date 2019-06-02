Arduino Code contains the code needed to be uploaded on Arduino. 
No extra libraries or dependencies are required for arduino except ros.h and std_msgs ones.
Arduino publishes filtered V and A on two topics /v and /a respectively. It also takes up messeges from topic "mode" which is for flash light. That topic is present in eklavaya4roboteq package.

bmsFinal.cpp is the name of the file for node named "bms". The node subscribes topics /v and /a and publishes state of charge on topic /SOC.

EXPLAINATION_________________________________________________________________________________________________________________________________________________________________________________________________
We have a function OCV = f(SOC)
OCV = a + b · (- lns )^m + c · s + d · e^(n ( s − 1 ))

OCV is calculated as OCV = V + A*R where R is the internal resistance of the cell.

R needs to be calculated using the method used in 12th class. 
Measure the no load voltage for battery using multimeter. Then connect connect shunt resistors along the battery and measure potential across the battery and current flowing through the shunt or simply the resistance of the shunt. (Current is better as resistance varies a lot with temperature). Update R in bmsFinal.cpp

Coefficients and constants for function f were approximated for the batteries kept at AGV lab. They were not calculated mathematically but were based on normal usage characteristics and how the graph should look. The approximations were done using desmos. https://www.desmos.com/calculator

SOC is calculated from the formula using newton raphson method.

USAGE________________________________________________________________________________________________________________________________________________________________________________________________________
1. Connect/Ensure the Current Sensor on pin A0
2. Connect/Ensure the Voltage Pin on pin A1(18 V MAX)
3. Give permissions to the Arduino using: sudo chmod 777 dev/tty*
4. Using the IDE, see the port on which arduino is connected
5. Run the Arduino node using: rosrun rosserial_python serial_node.py <PATH to USB PORT   would have found>
eg: rosrun rosserial_python serial_node.py /dev/ttyUSB0
6. Run the node: rosrun bms bms (After putting in a workspace and catkin_make)
7. See the topic /SOC for battery percentage: rostopic echo /SOC

Troubleshooting______________________________________________________________________________________________________________________________________________________________________________________________
1. The battery was modeled such that it 100% charged when its OCV is 13.2V. Any voltage above it will show that the battery is 100% charged. This will have to be changed by remodelling the graph such that at SOC = 1 potential is higher. Best way is to increase the value of 'c' So that the maximum voltage is increased and minimum remains same. One other way could be to increase the value of 'd' but it increases the lower potential a bit too. Then you will have to reduce the constant term a bit too. Don't change b or n much as they too drastic changes on shape of the curve. 
2. The battery was also modelled such that it is considered to be completely dicharged when V <=12

3.It was considered that Battery SOC for both batteries connected in series for motors is same. So voltage is measured for only one battery, current shall remain the same.

4. If the Battery SOC is too eratic, check topics /a and /v which one is iving more random results. Reduce the gain accordingly in current_and_voltageV3.ino file. Higher multiplicative gain gives higher response towards impulsive changes but too high value will increase noise too. Lower fixed gain reduces noise but reduces response to tiny changes in current. 
