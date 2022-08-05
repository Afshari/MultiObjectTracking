
## Multi Object Tracking 

- [Multi Object Tracking](#multi-object-tracking)
  - [1. Project Summary](#1-project-summary)
  - [2. Project Structure](#2-project-structure)
  - [3. Tools & Libraries](#3-tools--libraries)
  - [4. Run Application or Run Tests](#4-run-application-or-run-tests)
  - [5. Demonstration Video](#5-demonstration-video)
  - [6. TODO](#6-todo)
  - [7. References](#7-references)


### 1. Project Summary
The most important Algorithm for State Estimation is "Kalman Filter" and its variations, but "Kalman Filter" has weaknesses in an Environment with Clutter and Missdetection. <br /> <br />
So for this type of environment, there are other Algorithms like GNN, JPDA, MHT, and PMBM for object Tracking. <br /> <br />
I've implemented two types of Algorithms: "Single Object Tracking" in Clutter: NN (Nearest Neighbor), PDA (Probabilistic Data Association), Gaussian Sum, and "Multi-Object Tracking" in Clutter: GNN (Global Nearest Neighbor), JPDA (Joint Probabilistic Data Association), MHT (Multi Hypothesis Tracking). The programming language that I chose for this project is C++ (Qt Framework), the reason is that it is more efficient than other languages like Python. I've used QML for GUI, where users can draw a path, add measurements, and add clutters to the environment. <br /> <br />
I have to say that all 6 Algorithms that I've implemented are for the "Known Number of Objects," and there is no "born" or "death" in the environment. I am working on other Algorithms like PMBM for more dynamic environments with "born" and "death," but I don't finish them yet.

### 2. Project Structure
    ├── inc                         # Header Files
        ├── stats                   # StatsLib C++ library
    ├── libs                        # External Libraries
        ├── hungarian               # Implementation of Hungarian Algorithm in C++
        ├── murty                   # Implementation of Murty Algorithm in C++
    ├── src                         # Source Files
    ├── tests                       # Unit-Tests and Integration Tests
    ├── ui                          # QML Files for GUI

### 3. Tools & Libraries
~~~
Build Tool         : CMake
IDE 	           : Qt Creator
Language           : C++ 17
UI                 : QML
Unit Test          : Qt Test
Integration Test   : Google Test
Library            : Qt5
Library            : C++ Eigen Linear-Algebra library
Library            : StatsLib C++ library
Library            : Hungarian C++ Class
Library            : Murty C++ Class
~~~


### 4. Run Application or Run Tests
If you want to run the application or run the tests you have to set the "SET_RUN_TYPE" parameter of CMake. For running the application set it to "SET_RUN_APP" and for running the tests set it to "SET_RUN_TEST." You can do it via "Qt Creator" or you can set the parameter via the command line:
```
$ cmake -DSET_RUN_TYPE=SET_RUN_APP ..
```

### 5. Demonstration Video
You can watch the [Demonstration Video]() on the Youtube.


### 6. TODO
~~~
1. Add PMBM Algorithm
~~~

### 7. References
~~~
1. Multi-Object Tracking for Automotive Systems
   https://www.youtube.com/channel/UCa2-fpj6AV8T6JK1uTRuFpw
~~~