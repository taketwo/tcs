Installation
============

1. Install latest PCL from [source](https://github.com/PointCloudLibrary/pcl).
   Its dependencies include Boost and Eigen, make sure that Boost is at least
   **1.53** and Eigen is at least **3.2**. Also, a C++11 compliant compiler is
   required.

2. Clone this repository (recursively) and make out-of-source build:

```bash
git clone --recursive https://github.com/taketwo/tcs.git tcs
cd tcs
mkdir build
cd build
cmake ..
make
```

Data
====

There are several example RGBD scenes in the 'data/' folder. They include ground
truth segmentation.

Usage
=====

Navigate to the 'data/' folder and run

    ../bin/app_random_walker_segmentation test1.pcd

The program will load given file and proceed by building a graph of the input
point cloud. It will then display the graph (as a voxelized point cloud) so
that the user may select seed points. After the points are selected it will
perform random walker segmentation and visualize the results.

Visualizer interface
--------------------

This is the standard PCL visualizer with several extensions. It has a list of
objects available for visualization. To see it press `h`. The list will contain
status indicators, short descriptions, and keys that are used to toggle display
of the objects. For the random walker segmentation app it may look as follows:

                       Visualization objects
    ─────┬╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┬──────
      ☐  │ Input point cloud                              │ i
      ☐  │ Graph vertices                                 │ v
      ☐  │ Vertex curvature                               │ C
      ☐  │ Vertex normals                                 │ n
      ☐  │ Adjacency edges                                │ a
      ☐  │ Random walker seeds                            │ S
      ☒  │ Object clusters                                │ c
    ─────┴╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┴──────

For example, press `a` to toggle graph adjacency edges display.

Seed selection
--------------

Seed selection works as follows:

1. Hold shift and click a point. A randomly colored square will appear to confirm
   the selection. Select more points it you wish, they will all share the same
   label.
2. Press escape to start selecting points for the next label.
3. Repeat steps 1-2 until you selected point(s) for each object you want to
   segment in the scene.
4. Press espace once again.

Command-line options
--------------------

There are a number of command-line options, you may check it if you run the
program without passing any parameters. For example, you may try to change the
voxel resolution with `-v` option:

    ../bin/app_random_walker_segmentation test1.pcd -v 0.01

Another useful option is `--save`, which enables saving produced segmentation
into 'segmentation.pcd' file in the working directory. You may then use
`segmentation_evaluation` program to compare it with the ground truth.

Evaluation
==========

If you saved produced segmentation you may compare it with the ground truth
(which is available for the dataset in this repository). Simply run:

    ../bin/segmentation_evaluation segmentation.pcd test1.pcd --gui

The program will print the number of mis-labeled points to the console and
visualize them. As usual, you may press `h` to see available visualization
objects.

Documentation
=============

Generate the project documentation with the following command:

    make doc
    
Alternatively, the documentation could be accessed online at the
[GitHub pages](http://taketwo.github.io/tcs/index.html) of this repository.
