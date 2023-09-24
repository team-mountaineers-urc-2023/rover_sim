# rover_sim
Basic Gazebo simulation of Wanderer rover

To use run the following command:

`roslaunch rover_sim sim.launch`

Doing so will:
* Launch gazebo 
* Spawn in a basic rover model
* Launch 4 instances of Aruco Detect
* Create transforms from each camera to the base link
* Spawn in all 5 aruco markers

To adjust starting positions of aruco markers you can modify the `sim.launch` file. There are two arguments for each aruco marker that define
their X and Y values:

```
<arg name="x" value="X.XX" />
<arg name="y" value="Y.YY" />
```

By replacing `X.XX` and `Y.YY` with the X and Y positions in meters, you can change where the markers start

The markers are held in place, but driving into them may cause strange behaviors

The file `mavros_spoof.launch` is used to allow autonomy to work properly with the sim. It takes the (X,Y,Z) coordinates from Gazebo and
transforms them into Latitude, Longitude, and Height values that Autonomy can understand. These are run together by a higher level launch file
`wanderer_missions/launch/sim/sim_autonomy.launch` but can be run simultaneously from the terminal to achieve the same effect
