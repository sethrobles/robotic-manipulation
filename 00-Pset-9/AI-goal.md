This document is laying out my goals for an intended assignemtn in robotic manipulation

High level goals: Have a simulation that is able to demonstrate a Kukua IIWA 7 catch a falling object out of the air, first with pure knowledge of the system, and then if possible, catch it while using geometric estimations from cameras.

Step 1: Set up Imports (This can be done with the files that I'm already looking at, should not really include new imports without being asked beforehand)

Step 2: Set up Meshcat. Launch it.

Step 3: Save an SDF of an initial (or a box), like a cheddar box.

Step 4: Set up an environment in a URDF/SRDF file that consists of a Kuka iiwa 7 and the object to catch. Have the starting position of the catching thing be something that I could potentially change in the future. It should be a variable.

Step 5: Set up a hardware station or other systems that will automatically include the iiwa and then something for the box that will track its estimated center of mass and orientation. Set up a simulation / visualization system that will be bale to output an expected trajectory based on gravity. There will be no acceleration in anything but the z direction (bc of gravity).

Step 6: Set up an aribtary height at which we want to catch the "thing" at. Also, will have to work on catching parameters (it should be the box thickness + .1). Generally the catching system might be hard. I think we should have sufficient width to not catch the item as it falls down, but then start closing as soon as the bottom passes. Okay, so we'll work from the bottom actually so that it works with the camera. We will try to define a rectangle that marks the bottom of the box. this will then be used to determine the bottom position of the box in 3d coordinates. The box should be flat hoizontal (not angled) basically the whole time so it'll be somewhat easy to capture.

Catching involves setting up a trajectory. This doesn't really need to be good.

Step 7: Run the simulation (Hopefully the iiwa catches the object). The simulation/system will have to work in a weird way. It will have to assign a system to the falling position and then getters from this. It should be able to get the current position, and then generate a list of expected future positions (a trajectory) in the xyz domain that correspond with a specific time (after the start). This type of system should be compatible with the cameras and the type of pose estimation that we'll get later. Also close the gripper. We should probably be doing this force/position wise. Make sure that there's enough force to where the thing doesn't fall down any more once it's in the gripper.

Step 8: Display the simulation in meshcat.

// Everything under this is optional, something that could be done.

Step 9: Try running a random position of the objec when it's loaded. Basically just copy the scene from above, but make the position something random that's decently close to the robot in terms of x and y (but hopefully not directly above). It should be at a similar height.

Step 9: Now try to use a camera system to estimate geometric position, velocity, and trajectory. This means that the system set up before should work well so that it works with the computers. This means figuring out a geometric conversion from the cameras to the box. This may involve figuring out mesh things and making sure we can get a solid geometry / representation of the image.

Step 10: Run a simulation where there's no controller, it's just the box falling, make sure that the trajectory of the bottom matches what we expect.

Step 11: Run the control with this simulation.
