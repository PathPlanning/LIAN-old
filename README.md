# LIAN
Algorithm for 2D and 3D grid based path-finding with restriction on maximal turn angle in trajectory.
Abstract containing algorithm details can be found [here](https://arxiv.org/pdf/1506.01864.pdf)

Description
==========
Project contains implementation of 3D DLIAN.

Build and Launch
================
>To build the project you can use QtCreator or CMake. Both .pro and CMakeLists files are available in the repository. 
>Notice, that project uses C++11 standart. Make sure that your compiler supports it.
>To launch the compiled file and get a result you need an instance - an input XML file. You can find it in Examples folder. 

Input and Output files
======================
>Both files are an XML file with a specific structure. 
>Input file should contain:
>- Mandatory tag <b>\<map></b>. It describes the environment.
  * **\<height>** and **\<width>** - mandatory tags that define size of the map. Origin is in the upper left corner. (0, 0, 0) - is upper left, (*width*-1, *height*-1, 0) is lower right.
  * **\<startx>**, **\<starty>** and **\<startz>** - mandatory tags that define horizontal (X), vertical (Y) and height (Z) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width*-1], for *starty* - [0, .., *height*-1].
  * **\<finishx>**, **\<finishy>** and **\<finishz>** - mandatory tags that horizontal (X), vertical (Y) and height (Z) offset of the goal location.
  * **\<altitudelimits>** - optional tag that defines minimal allowed height of moving in attribute "min" and maximal allowed heighed in attribute 'max'. If 'max' is not defined, agent can move indefinitely high. If 'min' is not defined agent minimal height is 0.
  * **\<grid>** - mandatory tag that describes the square grid constituting the map. It consists of **\<row>** tags. Each **\<row>** contains a sequence of integers separated by blanks. Each number defines the height of obstacle on this cell "0" means that cell is fully traversable.
  * **\<title>**, **\<URL>**, **\<coordinates>**, etc - optional tags containing additional information on the map.
>- Mandatory tag <b>\<algorithm></b>. It describes the parameters of the algorithm.
  * **\<weight>** - optional tag. Defines the weight of heuristic function. Default value is "1".
  * **\<breakingties>** - optional tag. Defines the priority in OPEN list when nodes have the equal F-values. Possible values - "g-min", "g-max". Default value is "g-max".
  * **\<linecost>** - optional tag. Defines cost of straight move between two neighbour (be edge) cells. Default value is 1. 
  * **\<anglelimit>** - mandatory tag. Defines the maximal turn angle in trajectory.
  * **\<distance>** - mandatory tag. Defines length of path section. If using DLIAN (by setting tags **\<distanceMin>** and **\<decreaseDistanceFactor>**) defines the maximal length of path section.
  * **\<distanceMin>** - otional tag for using in DLIAN. Defines minimal path section length for DLIAN algorithm.
  * **\<decreaseDistanceFactor>** - otional tag for using in DLIAN. Defines the factor, on which DLIAN tries to decrease path section length during search. It creates list of possible distances, started from **distance**, decreasing each time by **decreaseDistanceFactor** and finished by **distanceMin**.
  * **\<pivotCircleRadius>** - optional tag. If value > 0 algorithm checks the "safety zone" (circle with defined radius) around every turn point for obstacles. If there are obstacles in safety zone, there is a risk that agent during maneuver will collide with obstacle. 
>- Optional tag <b>\<options></b>. Options that are not related to search.
  * **\<loglevel>** - Optional tag. Defines the level of detalization of log-file. Default value is "1". Possible values:
    * "0 - log-file is not created.
    * "1" - All the input data is copied to the log-file plus short **\<summary>** is appended. **\<summary>** contains info of the path length, number of steps, elapsed time, etc. **\<path>** tag is appended. It looks like **\<grid>** but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: **\<hplevel>** and **\<lplevel>**. **\<lplevel>** is the sequence of coordinates of cells forming the path. **\<hplevel>** is the sequence of sections forming the path.
    * "1.5" - *1*-log plus the information (explicit enumeration) on last iteration's OPEN and CLOSE lists.
    * "2" - *1*-log plus OPEN and CLOSE lists are written into the log-file after each step of the algorithm. Can make log-files really huge.
  * **\<logpath>** - Optional tag. Defines the directory where the log-file should be written. If not specified directory of the input file is used. 
  * **\<logname>** - Optional tag. Defines the name of log-file. If not specified the name of the log file is: "input file name"+"_log"+input file extension.
