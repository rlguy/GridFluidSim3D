This program is an implementation of a PIC/FLIP fluid simulation based on methods described in Robert Bridson's "Fluid Simulation for Computer Graphics" textbook.

Features include anisotropic surface reconstruction, 'LEGO brick' surface recontruction, and spray, foam, and bubble particles.

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot03.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot00.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot01.jpg)

![alt tag](http://rlguy.com/gridfluidsim/images/screenshot02.jpg)

This fluid simulation program generates a triangle mesh for each frame and stores this data in the .ply file format. The fluid simulation is configured in the file [src/main.cpp](src/main.cpp) and the default simulation drops a ball of fluid in the center of the fluid domain.

To render the simulation into an animation, you will need to import the series of .ply meshes into a rendering program where you can set up the scene/lighting/camera. I use the free and open source [Blender](http://www.blender.org) software to render the fluid simulations that I run.

If you are not familiar with Blender, you can try out this [.blend](https://drive.google.com/file/d/0B1bzpKpnt4f4LVBLUGZtVDZsaWc) file that will import the .ply meshes and render the default simulation. I have highlighted the important areas for configuring the render in this screen shot:

![alt tag](http://rlguy.com/gridfluidsim/images/blender-default_sim_render_screenshot.jpg)

To render the animation your will need to

1. Change the WATER_FILEPATH_DIRECTORY variable to the filepath of your bakefiles folder
2. Set the resolution and frame range of the animation
3. Set the output destination
4. Click the 'Run Script' button
5. Click the 'Animate' button