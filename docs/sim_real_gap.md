This doc holds some bullet points on the troubles with sim real

see "readme_why_planning_fails_on_real_system" at: docs/vid/Real/___this the good ......"
here is some info we used to improve the exploration.

additionally: replanning path (we have to account for walls we previously did not detect or dynamic obstacles that move out of the way, this is simply done by ocassionally replanning the path to the goal in a set number of intervals) 

the end goal does not get replanned unless it is unreachable , only the waypoint towards the goal get continuously replanned

what else: adjust robot speed 
speed in sim feels a lot slower then real robot

reduce robot speed to controller / mapping loop can keep up, slower robot -> less drift more accurate map
this and lidar data noise was already considered in sim and held up quite nicely (we used noise in the sim test env.)
since we operate a rather slow robot this actually transfered well.

controll loop is not that fast / optimized. the faster the robot runs (even in simulation) the more drift we have this effect can be quite easily archived even in sim
-> lots and lots of artifacts

also .....
