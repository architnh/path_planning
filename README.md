
# Comparison of Dijkstra's and A* for path planning.
## Overview :
Repository for comparison of multiple path-planning approaches like Dijkstra's, A*. This was part of the MEAM 6200 Advanced Robotics class by Dr. M. Ani Hsieh at University of Pennsylvania. While planning for quadrotor motion in cluttered environments, it is necessary to study muliple approaches for better path planning. Following is a comparison between two approaches.

<img src=docs/dijkstras.gif height="350" width="350" /> <img src=docs/astar.gif height="350" width="350" /> 
Figure 1: Dijkstra's algorithm vs A* algorithm implemented in a cluttered environment.

<p></p>

## Dijkstra's Algorithm Pseudocode:

``` 
function Dijkstra(Graph, source):                                                  
    dist[source] ← 0               
                                                                                
    create vertex priority queue Q                                                       
                                                                                     
    for each vertex v in Graph.Vertices:                                                                                                                                               
            cost[v] ← INFINITY       

    cost[source] = 0
    Q.add_with_priority(cost[source], source)  // heap push                                        
                                                                                              
                                                                                              
    while Q is not empty and goal is not reached:                                                
        u ← Q.extract_min()                    // heap pop                            
        for each neighbor v of u:              // Go through all v neighbors of u                                               
            alt ← cost[u] + Graph.Edges(u, v)                                               
            if alt < cost[v]:                                                                                                  
                cost[v] ← alt                                                                                                  
                prev[v] ← u                                                                                                  
                Q.add_with_priority(alt, v)                                               

    while start not reached:
        parent ← prev[node]                    // start with goal node and iterate till start node   
        path.append(parent)                                                                             
    return path
```                                  

The pseudocode of Dijkstra's algorithm using heap queue that I used is listed as above. ([Source](https://en.wikipedia.org/wiki/Dijkstra's_algorithm)) &emsp;

## A* Algorithm Pseudocode:
``` 
function Dijkstra(Graph, source):                                                  
    dist[source] ← 0               
                                                                                
    create vertex priority queue Q                                                       
                                                                                     
    for each vertex v in Graph.Vertices:                                                                                                                                               
            cost[v] ← INFINITY       

    cost[source] = 0
    Q.add_with_priority(cost[source]+ constant*dist(start,goal), source)  // heap push                                        
                                                                                              
                                                                                              
    while Q is not empty and goal is not reached:                                                
        u ← Q.extract_min()                                              // heap pop                            
        for each neighbor v of u:                                        // Go through all v neighbors of u                                               
            alt ← cost[u] + Graph.Edges(u, v) + constant*dist(v, goal)                                               
            if alt < cost[v]:                                                                                                  
                cost[v] ← alt                                                                                                  
                prev[v] ← u                                                                                                  
                Q.add_with_priority(alt, v)                                               

    while start not reached:
        parent ← prev[node]                                              // start with goal node and iterate till start node   
        path.append(parent)                                                                             
    return path
```                

The pseudocode of A* algorithm using heap queue that I used is listed as above. The only different is that in addition to the pre-existing functions for dijkstra's and the cost evaluation, heap algorithm, there is an inbuilt distance heuristic that helps it to quickly converge in the direction of the goal with expanding less number of nodes than Dijkstra's.


<img src=docs/astar_dijkstra.png height="960" width="540" /> 
Figure 2: Path returned by Dijkstra's (green) vs A* (blue).


## Comparison:
Finally, a comparison between the two yielded some interesting results. Both algorithms were carried out in the same environment with different resolution of step sizes while expanding the nodes. The results show that as the resolution increases, the time needed goes up, and the number of nodes expanded go up for both approaches, but the difference between Dijkstra's and A* starts becoming more significant. 

<img src=docs/statistics.png height="960" width="540" /> 
Figure 2: Number of nodes expanded vs resolution [x,y,z] in meters.
<!-- 

# Motion Planning using RRT and RRT*

## Overview :

<img src=imgs/rrtvsrrtstar.gif height="360" width="640" > <p></p>
Figure 1: RRT* vs RRT implementation as visualized on RVIZ.

The goal of this lab project was to explore Sampling based algorithms: RRT and its variants. Through this project, we implemented a binary occupancy grid (0 for free, 1 for occupied), and processed the Hokuyu 2D Lidar scans at a refresh rate of 25ms.


## RRT Pseudocode:

![rrt_algo](imgs/rrt_algo.png)

The pseudocode of the basic version of RRT that we used is listed as above. As for RRT*, we used the pseudocode mentioned below.

![rrt_algo](imgs/rrt_star_algo.png)

## F1TENTH RRT vs. Generic RRT

In general, RRT is often used as a global planner where the tree is kept throughout the time steps. Whenever there is a new obstacle, and the occupancy grid changes, the tree will change accordingly. In our case, RRT is used as a local planner for obstacle avoidance. This is due to the fact that we don't have a well-defined starting point and goal point when we're racing on a track and we want to run continuous laps. In our implementation, we are only keeping a tree for the current time step in an area around the car. As speed is of the utmost importance (one update of laserscan every 25 ms), farster the process of finding nodes, and traversing the tree, faster is the implementation.

## RRT vs. RRT* 
A comparison of the output we obtained from RRT* and RRT algorithm that we implemented is shown below.<p></p>


### RRT* introduces two major improvements over RRT - <p></p>
1) Each node is associated with its cost. This is a function which measures the distance from the goal point. The goal of the algorithm is to optimize the tree and reduce this cost. <p></p>
2) At each iteration, the vicinity of node being sampled is checked for nearby nodes on the tree which can be rewired to reduce the path traversed. If such nodes exist, they are rewired and the net cost of reaching the node is reduced. 

### Videos- <p></p>
[RRT](https://youtu.be/u7Lv9G6eQF8) &emsp;
[RRT*](https://youtu.be/NfnwbfQKN34) <p></p>

Because of the way the algorithm works, it goes on improving itself with every iteration. This results in a much cleaner, smaller path from the start to the target. Because the distance between two nodes is kept static and predefined for our problem, we can say with certainty that the number of intermediate nodes connecting the start to the goal is much lesser in RRT* as against RRT. RRT* also plans a straighter and cleaner path which is not haphazard. This allowed us to significantly increase the speed of the car during loop racing. It reduced the lap time significantly. We observed a considerable difference in the performance on straights and turns.
## References:
https://arxiv.org/pdf/1105.1186.pdf

 -->
