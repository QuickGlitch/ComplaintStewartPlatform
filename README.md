# ComplaintStewartPlatform
The Design and Control of a Humanoid Robotic Torso using Matlab

## Wheelchair Stewart Platform Examples
There are 2 examples: StewartPlatform and WheelchairModelWithSpine. Both should be run from the .m file, not the .slx (simmechanics file). The .m files should launch mechanics explorer which will animate the simmechanics 2G. Made in Matlab v.2015b. later versions of Simmechanics are not guaranteed to work.

NOTE: these examples are smaller, limited cases. PID control and Torque loads are excluded from these models. Uploading the complete models is on the to do list.

## GenerateEquationsofMotion
Generate the equations of motion, which can be used to solve the forward dynamics of the modified Stewart Platform.


## StewartPlatform
This is an example of a Stewart Platform with added springs in series. The main file runs the simulation using my equations (script) first and then, if a simechanics comparison is defined (see define variables, ==1 by default), a Simechanics equivalent is run and the results are compared (also simulation times). There is no spine, and the top platform is a simple cylinder.
The case is an arbritary test case with arbritary spring stiffnesses, damping, applied loading (a linearly increasing horizontal force applied at the top).

A final figure and animation will be generated after both simulations, comparing them and animated the manual/script simulation (replays 3 times). In the figure, the SM legend is the simmechanics result which should be on top of the manual result (so the manual will not be visible).


## WheelchairModelWithSpine
This is the final model I developed in Simmechanics. There is a spine, actuator locking (via triggered springs at the actuator ends) and a controller. The controller aims to keep the platform/body straight up. An arbritary loading is applied for half of the simulation time, a linear increasing horizontal force at the chest. In the second half of the simulation the force is abruptly gone, so the trunk swings back due to the PID and elastic components, then resettles at the desired position.
