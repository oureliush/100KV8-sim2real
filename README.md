This repo contains the code used to run sim2real on my robot 100KV8, a 2 Jointed Robotic Leg with an actuator on the knee and foot rotation joints.
The policy is a PPO trained in IsaacLab. And it was trained to do one thing, balance the robot using only torque commands to its joints.
And it succeeded. The robot, policy, and this code is a result of about a 2 years worth effort to dive deeper into the world of robotics and engineering. 
I made that decision around my freshmen year of high school after I finished my 3D remote control Arduino car with the spare parts of the robots that my school once built, as I was tired with dealing with things you could buy in kits and stuff.

Anyways enough of the backstory. 
This ran on a Raspberry Pi 4 4gb running Ubuntu 22.04 with a USB - CAN Adapter connecting the RPI directly to the motor controllers and IMU. 
I configured the RPI to have one isolated CPU to run the script on to reduce jitter significantly to have a stable 50 hz control loop. No realtime kernel used here.
AS WELL AS DISABLING THAT PESKY UNATTENDED UPGRADES PROCESS. if you couldn't tell, I actually lost my mind trying to figure out why the PI's control frames would become unstable after a long time of not using the pi.

I'll eventually open source the robot, actuators, and anything else custom that I designed for this robot along with all the build instructions. But just like with using GitHub. documenting what I'm doing is somehow a lot harder than actually doing it. 

Thanks for reading, and I hope this code helps someone out with their own sim2real project one day.
