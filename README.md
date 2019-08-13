Particle Filter Demo
====================

Introducing FlatEarth1, an autonomous rover built using the Lego NXT kit with two motors and one ultrasonic distance sensor.  It was designed to explore the planet Coffeetable, a 120 cm by 60 cm flat surface with cliff-like edges.

![Alt text](img/web_view1.JPG?raw=true "Robot")

![Alt text](img/web_view3.JPG?raw=true "Robot")

![Alt text](img/web_view4.JPG?raw=true "Robot")


The goal is to do localization using a particle filter.

- A particle filter with 50,000 particles is initialized to random headings and locations on Coffeetable.  Each particle represents a hypothesis about where the robot's midpoint between the driving wheels is located.
- The rover is placed in an unknown location and orientation on the surface of Coffeetable.
- Loop:
    - The rover moves forward until it senses the edge of the table, then pulls back a bit.  
    - Each particle in the filter also "moves" forward until it intersects the table edge.  
    - The difference between the rover's forward distance and that of the particles is used to compute importance weights.
    - Both the rover and the particles are steered to a new heading - this steering causes a position offset which is also modeled by the particles.
    - After the first iteration, particle count drops to 5000.

Some issues I ran into along the way:

- Using the nxt_python, the motors are driven by simplistic closed loop controllers running on my laptop, sending commands to the Lego brick (and receiving rotary encoder data) over Bluetooth.  So there's a lot of motion noise and uncertainty due to Bluetooth latency.
- My first design had a castoring third wheel in the back, which caused the controllers to lurch forward.  Easy fix was to fix the third wheel to steer forward only: whenever the robot is steered to new heading, it simply drags the back wheel across the table surface.
- Lots of empirical tweaking of the geometry.
- Since it has only one sensor (front and center), if the robot approaches the edge of the table at a shallow angle it may fall off before sensing the edge.  Simplest fix would be to use two sensors, but I cheated and set the initial orientation and steering angles to avoid falling off.

Here's the best position estimate achieved by the particle sensor.  Due to the symmetry of the table, both locations are equally likely!

![Alt text](img/filter.png?raw=true "Robot")

Video of the whole sequence (click to watch):

[![Particle Filter Demo](img/robot_video.png?raw=true)](https://vimeo.com/344785399 "Robot Demo 2 - Click to Watch!")
