# ROS
## **Part 1: Draw character with turtles!**
_In this project, I want you to draw some characters with turtlesim turtles. As you know turtlesim is the 2D simulator of ROS which gives you plenty of options to implement your ideas or develop current algorithms. In this simulator, there is a controllable turtle that is the main element of this project._
_For each of you, two characters have been specified which are available in the attached Excel file. For this part of your project use as many turtles as you need to form the specified characters. A sample is provided in the Figure 1._

![image](https://github.com/user-attachments/assets/8be1c52e-b833-4190-902e-3d5f4a03f8a1)

***

My characters were J, θ.

Code description:

spawn_turtle function:

This function creates a new turtle at the position specified by (x, y) and the angle theta and gives it a specific name. First, it waits for the spawn service to be ready and then spawns the turtle. If the service fails, it logs an error message.

function kill_turtle:

This turtle function deletes the entity with the given name. First, it waits until the kill service is ready, and then it deletes the turtle. If the service fails, it logs an error message.

main function:

First, it creates a new node, then deletes the default turtles created by turtlesim, then specifies different positions to form the two characters J, θ.
Then it creates the turtles in the said positions to form the characters and adds their names to the turtle_name list.
Finally, it stores the names of all created turtles in the vertex parameter service.

**Output:**

![image](https://github.com/user-attachments/assets/b468841b-0e2c-4836-80dd-1d28f17bc23c)

## **Part 2: Rotating the turtles!**

_Well, in this part, let’s do some exciting performance. As mentioned before, your turtle is controllable and you can rotate or move it by sending the proper topic. Use the rostopic list and explore related topics for linear and angular velocity. Write a Python script that rotates all turtles on the screen at the same time. Name this node as ”rotating_node”. Explain your approach and name all topics and types of messages that you used for this part._

_**Bonus:**_
_Write a Python script that makes it possible to use the arrow keys to move the first displayed letter in the turtlesim playground._

***

function rotate_turtles:

First, it creates a new node called rotating_node, then retrieves the names of the turtles stored in the ROS parameter server. If this parameter exists, it will get the names of the turtles and display them in the log. Otherwise, it logs a warning message and the function terminates.
Next, it creates a publisher for each turtle that sends Twist messages to the /name/cmd_vel topic. These publishers are stored in a dictionary called publishers.
In the next section, it sets the propagation rate of the messages to 10 Hz and creates a Twist message that has an angular velocity of 1.0. This angular velocity causes the turtles to rotate.
In the While loop, the Twist message is sent to all turtles as long as the ROS node is not shut down. For each turtle, a Twist message is issued and a log message is displayed. The program then pauses for the specified amount of time (0.1 seconds) to maintain a rate of 10 Hz.

Main function:

In this section, if the program is executed directly, the function rotate_turtles is called. If an exception of type ROSInterruptException occurs (for example, when the user stops the program by pressing Ctrl+C), the program terminates without error.

**Output:**

![image](https://github.com/user-attachments/assets/a8df3827-0367-4528-95e9-1b705d6d86e2)

**Bonus section:**

![image](https://github.com/user-attachments/assets/71ed32a2-f3bf-4e65-b51a-92b27b4f8d93)

## **Part 3: Launch files and Report**

_Note that this part is the most important section of your project and if it doesn’t have a launch file, YOUR PROJECT WILL NOT BE SCORED. So, to run your project, write a launch file that contains all required nodes for the complete operation of drawing letters. By running this launch file, the turtlesim_node should be executed and your assigned letters appear. After that, all turtles should start rotating till ctrl + c stops the process. Beside this launch file, write a summary of what you did to solve the challenges that you faced during this project. Your report should contain a complete explanation of all message types, services, and topics that you used and also plot the graph of all nodes. It is necessary to have an ”Error” section in your report. Make sure you explain all the problems you had and how you fixed them in your report. This will help you get a better score. Don’t forget to talk about the mistakes you made. And make sure to give clear instructions on how you solved each problem._

***

We create a folder called launch and create a launch file in it, now by opening the launch file in VS Code, we write the relevant codes in it and then type the execution command i.e. roslaunch demo_pkg run_rotate.launch:

**Output:**

![image](https://github.com/user-attachments/assets/24abe29d-c9c8-4634-926d-6c629f172ca6)

**Graph nodes:**

![image](https://github.com/user-attachments/assets/f18b8651-2db9-4512-a3cb-1a1dbbada782)


### Errors we encountered:

• While running roscore, we encountered an internet connection error, which was due to the internet outage.

• While running the rosrun command to run the codes, we encountered a package error, which we solved by running the source catkin_ws/devel/setup.bash command.

• During the execution of the code, we encountered the error of TurtleSim not running. We should have run TurtleSim and connected to its server before executing the code.

• In the first part of the turtle code, there was a default wire when running turtle, which we fixed with the kill_turtle function.

• In the second part of the code, we encountered a problem to rotate the turtles, we solved this problem by saving the names of the turtles and executing them in the next part, and the turtles started to rotate.

• The pygame library was not installed for the privilege section and we installed it through the VS Code terminal.

• While installing Ras, we encountered a problem that some files were not downloaded due to the internet being cut off, this problem was solved by reinstalling and downloading the incomplete files.

• In the last part, to execute the launch command, we encountered a problem that the turtles were printed but did not rotate, this problem was due to the Linux space being filled, and by emptying the Linux space, this problem was solved, and the turtles were rotating at the same time as they were printed.

• When executing RasLaunch, the turtles were only printed but did not rotate, this is because the turtles were given a rotation command before they were fully printed (because three commands are executed at the same time in the launch) by putting a delay at the beginning of the function of the second part of the code. (time.sleep) can solve this problem because this function waits until the codes of section 1 are fully executed, then this section is executed. For me, after running the turtles several times, they were printed quickly, and I no longer needed to use this function.

**Types of ROS messages, services, and topics**

1. Message Types: The project may use standard ROS message types such as std_msgs/String or custom message types as needed.
2. Services: These may include service types such as std_srvs/Empty or custom service definitions for processing specific requests.
3. Threads: ROS threads are probably used for communication between nodes. Common threads can include /cmd_vel for speed commands or custom threads for specific data exchanges.
