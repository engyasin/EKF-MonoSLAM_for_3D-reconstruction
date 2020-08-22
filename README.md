# EKF-MonoSLAM for 3D reconstruction

EKF stands for Extended Kalman Filter and MonoSLAM stands for Monocular Simultaneous Localization and Mapping.
This project is the first part for the code of a complete project for 3D reconstuction, the second part source code is [here](https://github.com/engyasin/3D-reconstruction_with_known_poses). the output from this part should be taken as input for 3D reconstruction (second part).

For the MonoSLAM part, the project from [here](https://github.com/rrg-polito/mono-slam) was used as backbone to this work.

*Notes:*
- for some of the jacobian calculation there's a Jupyter notebook explaining the result with the use of **Sympy** , to confirm the used formula, [here](https://github.com/engyasin/EKF-MonoSLAM_for_3D-reconstruction/blob/master/mono-slam/reference/Sympy_monoslam_jacobians_1.ipynb)

If you use this project in your work, you can cite this reference here [bib]:

```
@Inbook{Yousif2021,
      author="Yousif, Yasin Maan and Hatem, Iyad",
      editor="Koubaa, Anis",
      title="Video Frames Selection Method for 3D Reconstruction Depending on ROS-Based Monocular SLAM",
      bookTitle="Robot Operating System (ROS): The Complete Reference (Volume 5)",
      year="2021",
      publisher="Springer International Publishing",
      address="Cham",
      pages="351--380",
      doi="10.1007/978-3-030-45956-7_11",
      url="https://doi.org/10.1007/978-3-030-45956-7_11"
}
```

## How to install and make
This is a ROS project and it can be installed simply by placing the files in the source directory of the workspace. it has the following requirements:

* ROS kintect or a newer.

* [Gazebo Simulation](http://gazebosim.org/).

* [Keyboard control package](https://wiki.ros.org/turtlebot_teleop)

* Other packages like LibConfig and Eigen3 for C++

## Usage

After putting the source in workspace and making the project with `catkin_make` open the following terminals:

### Terminal 1 (Simulation):

```bash

$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="../some_world9.world"
```
the `.world` file in the root of the workspace

### Terminal 2 (Keyboard control):
```bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```

### Terminal 3 (Kalman Filter):
```bash
$ source ./devel/setup.bash
$ roslaunch mono_slam_sim.launch
```

This `launch` file is in the root of workspace.

### Terminal 4 (sparse 3D reconstruction):
After runinng the previous command , the keyboard must be used to move the robot around the object.

When it ends moving, the images and *nodes_and_prjs.txt* file should be taken to the [second program](https://github.com/engyasin/3D-reconstruction_with_known_poses) directory.  From these results, a 5th terminal should be opened to run an optimzation process.

### Terminal 5 (Sparse bundle Adjustment):

SBA requires moving the files, *NodesProjs* and *PointsNew* to the ROS root directory (usually in `.ros` folder) and then running
```bash
$ roslaunch sba_1.launch
```

### Terminal 6 (Dense 3D reconstruction):
Lastly after SBA process ends, *Nodes_Out.txt* should be renamed to *nodes_and_prjs.txt* and a 3d reconstruction program in dense mode is run by the command:

```bash
$ python main_3d.py
```

## Example Videos:
Many parts of the running of the project are recorded in short videos (less than 1 minute each) in this [playlist](https://www.youtube.com/playlist?list=PLKdJ5omea_pRlrw_EUnQFm7ZJqasIBBFa)

