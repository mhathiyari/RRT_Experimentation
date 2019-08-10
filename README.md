# RRT_Experimentation (WIP)
Rapidly Exploring Random Trees is one of the more prominent methods in the path planning world. It has many flavours and modifications some are hueristic while others are more general. When mature the goal of this repository is to serve as a nice reference Matlab code for these methods.   
This is also the place where I add different flavours and aprroached I experiment with.   
There is a sister repo with cpp code for most of this (WIP)   
   
   
    
## Basic RRT  
This assumes no dynamics constraints.  
Grid is a 5x5 unit  
Based on [Incremental Sampling-based Algorithms for Optimal Motion Planning](http://roboticsproceedings.org/rss06/p34.pdf)   
  
![Sample output from the Program](https://github.com/mhathiyari/RRT_Experimentation/blob/master/RRT_basic.png)
   
   
## RRTStar  
This assumes no dynamic constriants.Inlcudes rewire step.  
Grid is a 5x5 unit  
Based on [Optimal Kinodynamic Motion Planning using Incremental Sampling-based Methods](https://ieeexplore.ieee.org/document/5717430)   
Has the proofs of why rrtstar is garunteed to converge to optimal soln while regular rrt isn't  
    
![Sample output from the Program](https://github.com/mhathiyari/RRT_Experimentation/blob/master/RRT_Star_basic.png)



## RRTStar_Dynamics (Intergrated car dynamics with bicycle model  
   
It has dynamics of the vehicle inetegrated in to the planner.Hence only generates dynamically feasable trajectories.   
* The sampling space is 1000x1000 m 
* Speed is 10 m/s  
* Bicycle model of car dynamics is used.   
   
Based on [Path Planning using a Dynamic Vehicle Model](http://www.cs.cmu.edu/~motionplanning/reading/PlanningforDynamicVeh-1.pdf)    
   
   
![Sample output from the Program](https://github.com/mhathiyari/RRT_Experimentation/blob/master/RRT%20basic_dynamics.png)

