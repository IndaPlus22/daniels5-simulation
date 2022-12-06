# daniels5-simulation
A flocking simulation with red boids
# Principles
* <b>Seperation</b>: The boids will avoid clustering too close togheter
* <b>flocking</b>: While they avoid clustering too close they will still try to form groups
* <b>Alignment</b>: In the specific groups they will align the direction that they are traveling.

# Basic explenation
The program uses basic vector computation to determine the desired acceleration in form of a vector, it then uses the acceleration to compute the new velocity. The desired acceleration is computed as a sum of the 3 above principles

# How to run
<b>run </b>``cargo run``

# Known issues
* To close formation resulting in the seperation vector being too strong which sometimes make the boids fly into different directions.
