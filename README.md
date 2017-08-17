# LIAN
Algorithm for 2D and 3D grid based path-finding with restriction on maximal turn angle in trajectory.
Abstract containing algorithm details can be found [here](https://arxiv.org/pdf/1506.01864.pdf)

Description
==========
Project contains implementation of _2D LIAN_ and _DLIAN_ algorithm in __master__ branch and _3D LIAN_ and _DLIAN_ in __3D__ branch.

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
  * **\<height>** and **\<width>** - mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width*-1, *height*-1) is lower right.
  * **\<startx>** and **\<starty>** - mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width*-1], for *starty* - [0, .., *height*-1].
  * **\<finishx>** and **\<finishy>** - mandatory tags that horizontal (X) and vertical (Y) offset of the goal location.
  * **\<grid>** - mandatory tag that describes the square grid constituting the map. It consists of **\<row>** tags. Each **\<row>** contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" - for untraversable (actually any other figure but "0" can be used instead of "1").
  * **\<cellsize>** - optional tag that defines the size of one cell. One might add it to calculate scaled length of the path.
  * **\<title>**, **\<URL>**, **\<coordinates>**, etc - optional tags containing additional information on the map.
>- Mandatory tag <b>\<algorithm></b>. It describes the parameters of the algorithm.
  * **\<weight>** - defines the weight of heuristic function. Default value is "1".
  * **\<breakingties>** - defines the priority in OPEN list when nodes have the equal F-values. Possible values - "g-min", "g-max". Default value is "g-max".
  * **\<anglelimit>** - defines the maximal turn angle in trajectory.
  * **\<distance>** - defines length of path section. If using DLIAN (by setting tags **\<distanceMin>** and **\<decreaseDistanceFactor>**) defines the maximal length of path section.
  * **\<distanceMin>** - defines minimal path section length for DLIAN algorithm.
  * **\<decreaseDistanceFactor>** - defines the factor, on which DLIAN tries to decrease path section length during search.
>- Optional tag <b>\<options></b>. Options that are not related to search.
  * **\<loglevel>** - defines the level of detalization of log-file. Default value is "1". Possible values:
    * "0 - log-file is not created.
    * "1" or "short" - *0.5*-log plus **\<path>** is appended. It looks like **\<grid>** but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: **\<hplevel>** and **\<lplevel>**. **\<lplevel>** is the sequence of coordinates of cells forming the path (in case Theta* planner is used, this sequence is formed at post-processing step by invoking sequantually line-of-sight procedure on path's segments). **\<hplevel>** is the sequence of sections forming the path (in case planner other from Theta* is used, sections are formed at post-processing step using naive procedure).
    * "1.5" or "medium" - *1*-log plus the information (explicit enumeration) on last iteration's OPEN and CLOSE lists.
  * **\<logpath>** - defines the directory where the log-file should be written. If not specified directory of the input file is used. 
  * **\<logname>** - defines the name of log-file. If not specified the name of the log file is: "input file name"+"_log"+input file extension.
