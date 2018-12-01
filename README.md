#VRaytracer
![example image](Raytracer/image.png)

This is my version of a randomized raytracer. Instead of sending a ray at every pixel into the scene, it sends a ray at every pixel in a random direction. Therefore more iterations of random rays create a more visually stunning image. However since it needs more rays it also takes more time to complete processing. 

Materials were based off of Peter Shirley's algorithms for metal, glass and lambertian.

Features:
* Obj loading
* Mesh processing on OBJ files
* primitive processing (Triangles, planes, Plane bounds)
* Different materials
* Easy scene class to hold objects in the scene

since it takes too long to process on the computers, I would recommend lowering the resolution and running the program. run "make" to compile the required files and then run "./run" to start the raytracer. The script converts the ppm file into png using the "convert" function.

NOTE:
* If there is a linkage issue with Eigen, please try it on linux subsystem (but in most cases this should not happen)

* This is meant to be a working project on the CDF computers/ Linux

* Initial resolution will be low for convenience. You can set resolution yourself by going into raytracer.cpp and changed the R_WIDTH and R_HEIGHT accordingly and recompiling the program.


TO RUN:

run "run" shell script. The raytracer is already built with default low resolution settings
