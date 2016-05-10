# GridFluidSim3d

This program is an implementation of a PIC/FLIP fluid simulation written in C++11  based on methods described in Robert Bridson's "Fluid Simulation for Computer Graphics" textbook. The fluid simulation program outputs triangles meshes that represent the fluid surface for each frame in an animation. The triangle meshes are stored in the Stanford .PLY file format which can then be imported into your renderer of choice. 

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot03.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot00.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot01.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot02.jpg)

## Features
Below is a list of features implemented in the simulator.

* Isotropic and anisotropic particle meshing
* Diffuse particle simulation (spray, bubble, and foam particles)
* 'LEGO' brick surface reconstruction
* Save and load simulation states
* GPU accelerated fourth-order Runge-Kutta integration using OpenCL
* GPU accelerated scalar field computation using OpenCL
* Tricubic velocity interpolation
* Modified Incomplete Cholesky pressure solver
* Supported on Windows, OS X, and Linux

## Dependencies

There are three dependencies that are required to build this program:

1. OpenCL headers (can be found at [khronos.org](https://www.khronos.org/registry/cl/))
2. An OpenCL SDK specific to your GPU vender (AMD, NVIDIA, Intel, etc.)
3. A compiler that supports C++11

## Rendering

This fluid simulation program generates a triangle mesh for each frame and stores this data in the [bakefiles/](src/bakefiles) directory as a sequence of .PLY files. The fluid simulation is configured in the file [src/main.cpp](src/main.cpp) and the default simulation drops a ball of fluid in the center of the fluid domain.

To render the simulation into an animation, you will need to import the series of .ply meshes into a rendering program where you can set up the scene/lighting/camera. I use the free and open source [Blender](http://www.blender.org) software to render the fluid simulations that I run.

If you are not familiar with Blender, you can try out this [.blend](https://drive.google.com/file/d/0B1bzpKpnt4f4LVBLUGZtVDZsaWc) file that will import the .ply meshes and render the default simulation. I have highlighted the important areas for configuring the render in this screen shot:

![alt tag](http://rlguy.com/gridfluidsim/images/blender-default_sim_render_screenshot.jpg)

To render the animation your will need to:

1. Change the _WATER_FILEPATH_DIRECTORY_ variable to point to the filepath of your bakefiles folder
2. Set the resolution and frame range of the animation
3. Set the output destination
4. Click the 'Run Script' button
5. Click the 'Animate' button

Blender will render your animation as a sequence of images which you can then convert into a video with a tool such as [ffmpeg](https://ffmpeg.org/).



