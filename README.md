
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


<img src=docs/astar_dijkstra.png height="540" width="960" /> 
Figure 2: Path returned by Dijkstra's (green) vs A* (blue).


## Comparison:
Finally, a comparison between the two yielded some interesting results. Both algorithms were carried out in the same environment with different resolution of step sizes while expanding the nodes. The results show that as the resolution increases, the time needed goes up, and the number of nodes expanded go up for both approaches, but the difference between Dijkstra's and A* starts becoming more significant. 

<img src=docs/statistics.png height="540" width="960" /> 
Figure 2: Number of nodes expanded vs resolution [x,y,z] in meters.
