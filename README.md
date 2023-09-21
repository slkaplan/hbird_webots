# Mini-Project 1 Writeup

## Controller Implementation

Our controller implementation is in this repository. Here is a link to that file:


## Agent Control Node Implementation

Our agent_control_node is in another repository. Here is a link to that repository and file:
https://github.com/slkaplan/hbird_common/blob/main/hbird_navigation/hbird_navigation/agent_control_node.py

## Gifs of Motion

## Write-up

### What did you learn from this? What did you not know before this assignment?

We learned how to deal with and troubleshoot codebases that we did not write from scratch. This was additionally troublesome as we all had not worked with ROS in a while, so we spent a good deal of time relearning some ROS command-line debug tools. 
More importantly, we did not know any drone physics or PID controls prior to this assignment. While we probably could not write the Hummingbird simulator from scratch, we are pretty confident at implementing simple PID control loops.

### What was the most difficult aspect of the assignment?

The most difficult conceptual part of the assignment was figuring out how to tune the cascaded controllers. We knew that we couldn’t tune both controllers at once and had to tune them independently. But it was difficult conceptually to figure out which gains and setpoints to use to isolate specific parts of the controller to turn different things. We had many simulation runs of the drone flying around uncontrollably and crashing before we figured it out. 

### What was the easiest or most straightforward aspect of the assignment?

The easiest part of the assignment was writing the controller specifically for the z height. We had already written a controller for z height for the warmup and we could transfer it over pretty easily. In a similar vein, it was relatively straightforward to take the dynamics equations given to us in class and turn them into code for the other parts of the controller. 

### How long did this assignment take? What took the most time (Setup? Figuring out the codebase/ROS2? Coding in Python? Exploring the questions?)?

This assignment took approximately 10 hours. Most of the time was spent on debugging issues with the simulator and understanding the codebase. The actual writing of the control code went really quickly, especially after having done the warm-up. Tuning the controller was also relatively quick. But trying to understand which parts of the existing codebase did what and trying to debug issues when the simulator wouldn’t run correctly took up a lot of our time. 

### What did you learn about PID controllers that we didn’t explicitly cover in class or in this assignment?

We had an interesting conversation about specifically tuning the controller for the heading angles. The setpoint for our heading originally was 2pi. This meant, if the robot overshot 2pi, instead of the angle reading slightly over 2pi, it would wrap back around to reading just over 0. This meant that the robot would try to correct its error by turning the long way around the circle back to 2pi and would just spin in circles with no oscillation. PID controllers rely on oscillation to eventually reach the setpoint so we needed to factor this in when we set our heading setpoint. 

### What more would you like to learn about controls?

Sam: I would like to learn some of the derivations (from scratch). I know the point of the assignment was to be fairly plug-and-play, but I never truly remember/understand something until I am comfortable deriving it myself. 

I would also like to learn why we needed to adjust the heading in order for our controls algorithm to work!

Liv: I enjoy dynamics and understanding the math and physics behind why things move. I feel like I am comfortable writing a controller to control U. But I feel less confident that I understand how U is used to actually control the motion of the robot. I’d like to understand more about the link between the low level control of individual rotors and the higher level control of setpoints and positioning. 


