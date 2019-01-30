Homework 1
==========

Created three python nodes for ROS: Random Waypoint, Lawnmower Sweep,
and Expanding Square.

Overleaf link to report: https://www.overleaf.com/read/yfkqhyygcwhb

The bagfiles for all tests can be found at the link below:

https://drive.google.com/drive/folders/1OxkBHOT-Hb2OxJMV2XRGGcuJqOcw74yY?usp=sharing
\#\# Authors

Getting Started
---------------

This package requires Ubuntu 16.04 or later and Gazebo.

First set up the package in your environment. This can be done by
cloning the repository into a catkin workspace. Execute the command:

`git clone https://github.com/ml-robotics-cs189/pa2.git`

Then, still in the catkin workspace, run:

`>> catkin_make`

Once this is complete the launch files can be executed from anywhere in
the catkin workspace. Type:

`>>catkin_ws roslaunch/pa2 <filename>.launch`

The executable files contained in the launch folder are
expanding\_square.launch, lawnmower.launch, and
random\_waypoints.launch.

Running the tests
-----------------

Once the files have been launched, the behavior should begin
automatically.

### Random Waypoint

The boat will move toward a waypoint randomly selected in the given
region.

### Lawnmower

The boat will head to a corner where it will begin a lawnmower sweep to
the opposite corner.

### Expanding Square

The boat begins at the center and moves in larger and larger squares
outward to scan the region.



-   **Jennifer Jain**
-   **Monika Roznere**
-   **Evan Honnold**
-   **Amy Sniffen**

