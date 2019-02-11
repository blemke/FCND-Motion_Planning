# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)

## To complete this project on your local machine, follow these instructions:
### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```
### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project. 

### Inspecting the relevant files
For this project, you are provided with two scripts, `motion_planning.py` and `planning_utils.py`. Here you'll also find a file called `colliders.csv`, which contains the 2.5D map of the simulator environment. 

### What's going on in  `motion_planning.py` and `planning_utils.py`
In addition to MANUAL, ARMING, DISARMING, TAKEOFF, LANDING, and WAYPOINT, motion_planning also has a PLANNING state.

`motion_planning.py` is basically a modified version of `backyard_flyer.py` that leverages some extra functions in `planning_utils.py`. It should work right out of the box.  Try running `motion_planning.py` to see what it does. To do this, first start up the simulator, then at the command line:
 
```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python motion_planning.py
```

You should see the quad fly a jerky path of waypoints to the northeast for about 10 m then land.  What's going on here? Your first task in this project is to explain what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script, and how the functions provided in `planning_utils.py` work. 

### Overview of planner

The planning algorithm is looks something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. You can determine your home location from `self._latitude` and `self._longitude`. 
- Perform a search using A* or other search algorithm. 
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]). 

### Writeup

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
##### motionplanning.py
In addition to MANUAL, ARMING, DISARMING, TAKEOFF, LANDING, and WAYPOINT, motion_planning also has a PLANNING state.  plan_path function contains the logic to impement the planning.
##### planning_utils.py
###### create_gridcreate_grid(data, drone_altitude, safety_distance)    
Returns a grid representation of a 2D configuration space based on given obstacle data, drone altitude and safety distance arguments.
###### Action(Enum)
An action is represented by a 3 element tuple.  The first 2 values are the delta of the action relative to the current grid position. The third and final value is the cost of performing the action.
###### valid_actions(grid, current_node)
Returns a list of valid actions given a grid and current node.
###### a_star(grid, h, start, goal)
implements the eponymous pathfinding algorithm
###### heuristic(position, goal_position)
uses Euclidean / Frobenius norm


These scripts contain a basic planning implementation that includes...

And here's a lovely image from the simulator 
![Top Down View](./misc/high_up.png)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
This is done with the following lines in plan_path:

```python
        #read lat0, lon0 from colliders into floating point values
        colliders_filename = 'colliders.csv'
        with open(colliders_filename) as f:
            lat_str, lon_str = f.readline().split(',') 
            lat0, lon0 = float(lat_str.split(' ')[-1]), float(lon_str.split(' ')[-1]) 
        print('From colliders ' + str(lat0) + ' ' +str(lon0))

        #set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
Here we successfully determine your local position relative to global home you'll be all set. In plan_path:
```python
        #retrieve current global position
        current_global_position = [self._longitude, self._latitude, self._altitude]
 
        #convert to current local position using global_to_local()
        self._north, self._east, self._down = global_to_local(current_global_position, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. In plan_path:
```python
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        #print(grid)
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # convert start position to current position rather than map center
        grid_start = (int(np.ceil(self.local_position[0] - north_offset)), int(np.ceil(self.local_position[1] - east_offset))) 
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

First the start of the script is modified to accept GPS coordinates of the target location.  Also the timeout period is extended to 2 minutes.
```python
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lat', type=str, default='37.792863', help='goal latitude')
    parser.add_argument('--goal_lon', type=str, default='-122.399015', help='goal longitude')
    parser.add_argument('--goal_alt', type=str, default='0.0', help='goal altitude')

    args = parser.parse_args()

    global_goal_position = (float(args.goal_lon), float(args.goal_lat), float(args.goal_alt))
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=120)
    drone = MotionPlanning(conn, global_goal_position)
    time.sleep(1)

    drone.start()
```

Then in plan_path:
```python
        # adapt to set goal as latitude / longitude position and convert
        local_goal = global_to_local(self.global_goal_position , self.global_home)
        grid_goal = (int(np.ceil(local_goal[0] - north_offset)), int(np.ceil(local_goal[1] - east_offset)))
```


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Modified the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2).  Thus was done by expanding the number of possible Actions(Enum) from 4 to 8:

```python
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    SOUTHWEST =(-1,-1, np.sqrt(2))
    NORTHWEST =(1,-1, np.sqrt(2))
    SOUTHEAST =(-1, 1, np.sqrt(2))
    NORTHEAST =(1, 1, np.sqrt(2)) 
```

Also, the specifics the the directions were spelled out in valid actions

```python
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if y - 1 < 0 or x - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if y - 1 < 0 or x + 1 > n or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if y + 1 > m or x + 1 > n or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if y + 1 > m or x - 1 < 0 or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)

```


#### 6. Cull waypoints 
A collinearity test was used to prune the path of unnecessary waypoints. Explain the code you used to accomplish this step.  If any point in the path was found to be within a 1m tolerance of the path from the previous and next waypoint, it was removed.

```python
def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while (i + 2) < len(pruned_path):
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        p2_unnecessary = collinearity_float(p1, p2, p3)
        if p2_unnecessary:
            pruned_path.remove(p2)
        else:
            i += 1

    return pruned_path
```



### Execute the flight
#### 1. Does it work?
It works!  The default lat and long targets sucessfully navigate and avoid buildings.


