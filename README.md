# Programming Assignment 2

Extended the code from the first programming assignment so that the boat seeks out areas with high clorophyll measurements, using a Gaussian Process regression model to choose waypoints.

## Getting Started
This package requires Ubuntu 16.04 or later and Gazebo.

First set up the package in your environment. This can be done by
cloning the repository into a catkin workspace. Execute the command:

`git clone https://github.com/ml-robotics-cs189/pa2.git`

Then, still in the catkin workspace, run:

`>> catkin_make`

Once this is complete the launch files can be executed from anywhere in
the catkin workspace. Type:

`>>catkin_ws roslaunch/pa2 <filename>.launch`

## Specify Bag File Paths
Since the bag files are very large, we do not include them within our repository.  You can download them [here](https://drive.google.com/drive/folders/1OxkBHOT-Hb2OxJMV2XRGGcuJqOcw74yY?usp=sharing). Once the bagfiles are downloaded, you will need to create a `bagfile_location.json` and add to it your bagfile paths.  For example:

```
{"bagfile_paths": [
    "/home/f0024tn/cs89/catkin_ws/src/pa2/bagfiles/expanding_square_2019-01-29-18-07-18.bag",
    "/home/f0024tn/cs89/catkin_ws/src/pa2/bagfiles/lawnmower_2019-01-29-18-12-14.bag",
    "/home/f0024tn/cs89/catkin_ws/src/pa2/bagfiles/random_waypoints_2019-01-29-18-01-07.bag"]
}
```

## Running the tests

Once the files have been launched, the behavior should begin
automatically.