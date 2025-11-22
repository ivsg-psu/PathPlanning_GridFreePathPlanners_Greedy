# PathPlanning_GridFreePlanners_Greedy

<!--
The following template is based on:
Best-README-Template
Search for this, and you will find!
>
<!-- PROJECT LOGO -->
<br />
<p align="center">
  <!-- <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

  <h2 align="center"> PathPlanning_GridFreePlanners_Greedy
  </h2>

  <pre align="center">
    <img src=".\Images\PathPlanning_GridFreePlanners_Greedy.jpg" alt="main PathPlanning_GridFreePlanners_Greedy picture" width="960" height="540">
    <font size="-2">Photo by <a href="https://unsplash.com/@hcmorr?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Hanna Morris</a> on <a href="https://unsplash.com/photos/sign-illustration-_XXNjSziZuA?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font>
</pre>

  <p align="center">
    The purpose of this repo is to demonstrate a "Greedy" path planner.
    <br />
    <!-- a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/tree/main/Documents">View Demo</a>
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues">Report Bug</a>
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues">Request Feature</a -->
  </p>
</p>

***

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About the Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="structure">Repo Structure</a>
      <ul>
        <li><a href="#directories">Top-Level Directories</li>
        <li><a href="#dependencies">Dependencies</li>
      </ul>
    </li>
    <li><a href="#functions">Functions</li>
      <ul>
        <li><a href="#core-functions">Core Functions</li>
        <ul>
          <li><a href="#fcn_Greedy_greedyPlanner">fcn_Greedy_greedyPlanner - Core function of the repo, uses "greedy" algorithm to perform path plan</li>
        </ul>
        <li><a href="#basic-support-functions">Helper Functions</li>
        <ul>
        </ul>
      </ul>
    <li><a href="#usage">Usage</a></li>
     <ul>
     <li><a href="#general-usage">General Usage</li>
     <li><a href="#examples">Examples</li>
     </ul>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

***

<!-- ABOUT THE PROJECT -->
## About The Project

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com)-->



* Inputs:
  * a cost graph, or cGraph which is an NxN matrix of costs to travel from (row) then to (column) one of N points within a map.
  * a start and finish point. These are usually the 2nd-to-last and last point in the cost grapoh
* Outputs
  * the sequence of points that traverse from the start point to the finish point.
<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1. Make sure to run MATLAB 2020b or higher. Why? The "digitspattern" command used in the DebugTools utilities was released late 2020 and this is used heavily in the Debug routines. If debugging is shut off, then earlier MATLAB versions will likely work, and this has been tested back to 2018 releases.

2. Clone the repo

   ```sh
   git clone https://github.com/ivsg-psu/PathPlanning_GridFreePlanners_Greedy
   ```

3. Run the main code in the root of the folder (script_demo_Greedy.m), this will download the required utilities for this code, unzip the zip files into a Utilities folder (.\Utilities), and update the MATLAB path to include the Utility locations. This install process will only occur the first time. Note: to force the install to occur again, delete the Utilities directory and clear all global variables in MATLAB (type: "clear global *").
4. Confirm it works! Run script_demo_Greedy. If the code works, the script should run without errors. This script produces numerous example images such as those in this README file.

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- STRUCTURE OF THE REPO -->
### Directories

The following are the top level directories within the repository:
<ul>
 <li>/Data folder: This is the location where small data files are stored that are used by codes within this repo.</li>
 <li>/Documents folder: Descriptions of the functionality and usage of the various MATLAB functions and scripts in the repository.</li>
 <li>/Functions folder: The majority of the code for the point and patch association functionalities are implemented in this directory. All functions as well as test scripts are provided.</li>
 <li>/Images folder: Contains fig (figure), JPG, and GIF images for use in this README.md.</li>
 <li>/Installer folder: Contains an automated installer function that connects the local MATLAB instance to GitHub repos, installing dependency libraries and setting up workspace path to access these dependent functions.</li>
 <li>/Utilities folder: Dependencies that are utilized but not implemented in this repository are placed in the Utilities directory. These can be single files but are most often folders containing other cloned repositories.</li>
</ul>

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

### Dependencies

* [Errata_Tutorials_DebugTools](https://github.com/ivsg-psu/Errata_Tutorials_DebugTools) - The DebugTools repo is used for the initial automated folder setup, and for input checking and general debugging calls within subfunctions. The repo can be found at: <https://github.com/ivsg-psu/Errata_Tutorials_DebugTools>. This is automatically installed as the first repo by the auto-installer.

* [PathPlanning_MapTools_MapGenClassLibrary](https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary) - the MapGen library functions that generate test maps. The repo can be found at: <https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary>

* [PathPlanning_GridFreePathPlanners_VGraph](https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VGraph) - the VGraph library calculates the visibilty graph of a map: <https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VGraph>

* [PathPlanning_PathTools_PathClassLibrary](https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary) - the Path library contains tools used to find intersections of the data with particular line segments, which is used to find start/end/excursion locations in the functions. The repo can be found at: <https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary>

* [PathPlanning_PathTools_GetUserInputPath](https://github.com/ivsg-psu/PathPlanning_PathTools_GetUserInputPath) - the GetUserInput library contains simple functions to input points in a sequence. The repo can be found at: <https://github.com/ivsg-psu/PathPlanning_PathTools_GetUserInputPath>

* [FieldDataCollection_VisualizingFieldData_PlotRoad](https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad) - the PlotRoad library contains useful and advanced plotting tools, particularly useful for showing XY or LLA data. The repo can be found at: <https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad>

Upon running the demonstration code, dependencies are automatically updated and installed in a folder called "Utilities" under the root folder.

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- FUNCTION DEFINITIONS -->
## Functions

### Core Functions

#### fcn_Greedy_greedyPlanner

The function fcn_Greedy_greedyPlanner is the main code of this repo. It calculates a path through an obstacle field using the Greedy algorithm.

<pre align="center">
  <img src=".\Images\fcn_Greedy_greedyPlanner.png" alt="fcn_Greedy_greedyPlanner picture" width="400" height="300">
  <figcaption>Fig.1 - The function fcn_Greedy_greedyPlanner calculates a path through an obstacle field using the greedy algorithm.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

### Helper Functions

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- USAGE EXAMPLES -->
## Usage
<!-- Use this space to show useful examples of how a project can be used.
Additional screenshots, code examples and demos work well in this space. You may
also link to more resources. -->

### General Usage

Each of the functions has an associated test script, using the convention

```sh
script_test_fcn_fcnname
```

where fcnname is the function name as listed above.

As well, each of the functions includes a well-documented header that explains inputs and outputs. These are supported by MATLAB's help style so that one can type:

```sh
help fcn_fcnname
```

for any function to view function details.

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

### Examples

1. Run the main script to set up the workspace and demonstrate main outputs, including the figures included here:

   ```sh
   script_demo_Laps
   ```

    This exercises the main function of this code.

2. After running the main script to define the included directories for utility functions, one can then navigate to the Functions directory and run any of the functions or scripts there as well. All functions for this library are found in the Functions sub-folder, and each has an associated test script. Run any of the various test scripts; each can work as a stand-alone script.

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

## Major release versions

This code is still in development (alpha testing)

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- CONTACT -->
## Contact

Sean Brennan - [sbrennan@psu.edu](sbrennan@psu.edu)

Project Link: [hhttps://github.com/ivsg-psu/PathPlanning_GridFreePlanners_Greedy](https://github.com/ivsg-psu/PathPlanning_GridFreePlanners_Greedy)

<a href="#pathplanning_gridfreeplanners_greedy">Back to top</a>

***

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
