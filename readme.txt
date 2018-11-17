/// Compile and Run Instructions ///

In order to compile this project, simply enter the following command from the
directory in which the java code files are located:

    javac SensorNetwork.java

The program can then be run using the following command:

    java SensorNetwork

Once the program has started, a window will open for the display, and the only
further input that will be required from the user is to press enter in order to
proceed through the various steps of the simulation.

/// Information Regarding the Code ///

This Program consists of nine classes. Eight of these are of my own making. The
core class is the SensorNetwork class in which the various steps of the 
simulation are implemented and the other classes are utilized. The Sensor,
AdjacencyMatrix, and RoundObstacle classes implement objects that are key to
the simulation. Th objects that are implemented are evident based on the names
of the classes. The Parameters class holds values for parameters that will be
used referred to during the simulation. The algorithms class holds functions
for calculating various algorithms that are used during the simulation. The 
Sector and Display classes are for splitting up the window into parts and 
displaying objects within those parts, respectively. The StdDraw class is the
final class that is used and the only one that is not of my own making. This
is a very commonly used java class that provides one with a basic capability
for creating drawings within their programs.