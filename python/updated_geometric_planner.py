import math
import rospy
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

import planner_helpers as ph

VALIDITY_CHECKING_RESOLUTION_KM = 0.05


class GridStateSampler(ob.StateSampler):
    def __init__(self, space):
        super(GridStateSampler, self).__init__(space)
        self.name_ = "GridStateSampler"
        self.rng_ = ou.RNG()
        self.space = space

        # Store bounds of state space
        self.x_lo, self.x_hi = self.space.getBounds().low[0], self.space.getBounds().high[0]
        self.y_lo, self.y_hi = self.space.getBounds().low[1], self.space.getBounds().high[1]

        # Set grid density (n x n) on state bounds
        self.n = rospy.get_param('grid_n', default=15)
        rospy.loginfo("GridStateSampler using nxn grid with n = {}".format(self.n))

    def sampleUniform(self, state):
        # Sample random point from grid
        # idx = 0 means lo, idx = n = hi
        x_idx = self.rng_.uniformInt(0, self.n)
        y_idx = self.rng_.uniformInt(0, self.n)
        x = self.x_lo + (self.x_hi - self.x_lo) * x_idx / self.n
        y = self.y_lo + (self.y_hi - self.y_lo) * y_idx / self.n
        state.setXY(x, y)
        return True


def absolute_distance_between_angles(angle1, angle2):
    fabs = math.fabs(math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2)))
    return fabs


def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    elif plannerType.lower() == "rrtxstatic":
        return og.RRTXstatic(si)
    elif plannerType.lower() == "rrtsharp":
        return og.RRTsharp(si)

    # multi-query
    elif plannerType.lower() == "lazyprmstar":
        return og.LazyPRMstar(si)

    # single-query
    elif plannerType.lower() == "rrtconnect":
        return og.RRTConnect(si)
    elif plannerType.lower() == "lbtrrt":
        return og.LBTRRT(si)
    elif plannerType.lower() == "lazylbtrrt":
        return og.LazyLBTRRT(si)

    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


def indexOfObstacleOnPath(positionXY, nextLocalWaypointIndex, numLookAheadWaypoints, omplPath):
    '''Look numLookAheadWaypoints ahead on path, returns the waypoint index that has an obstacle when approaching it.

    Example:
       Let positionXY be between index 2 and 3 in the omplPath
       Thus, nextLocalWaypointIndex = 3
       Let numLookAheadWaypoints = 2
       Let B = sailbot. X = obstacle. Numbers = waypoint indices in omplPath.
       Scenario 0: 0     1     2  B X3     4     5     6     7  => Returns 3  (1 index forward from B)
       Scenario 1: 0     1     2  B  3  X  4     5     6     7  => Returns 4  (2 indices forward from B)
       Scenario 2: 0     1     2  B  3     4  X  5     6     7  => Returns -1 (Nothing btwn B->3->4, look ahead 2)
       Scenario 3: 0     1     2X B  3     4     5     6     7  => Returns -1 (Nothing btwn B->3->4, not look behind)
       Scenario 4: 0     1     2 XBX 3     4     5     6     7  => Returns 0 (B invalidState)

    Args:
       positionXY list [x,y]: the sailbot position in xy (km) coordinates in the omplPath coordinate system
       nextLocalWaypointIndex (int): The index in omplPath that the sailbot is currently going towards
       numLookAheadWaypoints (int): Number of waypoints to look ahead for obstacles. Must ensure that:
                                    numLookAheadWaypoints <= (omplPath.getLength() - nextLocalWaypointIndex)
       omplPath (OmplPath): Path that the sailbot is currently following

    Returns:
       int representing the waypoint index that has an obstacle when approaching it.
       -1 if there are no obstacles on path
       0 if positionXY starts at invalid position
    '''
    # Setup for obstacle-on-path checking
    solutionPath = omplPath.getSolutionPath()
    spaceInformation = omplPath.getSpaceInformation()
    firstState = spaceInformation.allocState()
    firstState.setXY(positionXY[0], positionXY[1])

    # Edge case: Boat is in an invalid state
    if not spaceInformation.isValid(firstState):
        print("WARNING: not spaceInformation.isValid(firstState)")
        return 0

    # Edge case: Empty path
    if len(solutionPath.getStates()) == 0:
        print("WARNING: len(solutionPath.getStates()) = 0. Expected >=1.")
        return -1

    # Get the relevant states (ignore past states and use current position as first state)
    relevantStates = []
    relevantStates.append((0, firstState))
    for i in range(numLookAheadWaypoints):
        stateIndex = nextLocalWaypointIndex + i
        relevantStates.append((stateIndex, solutionPath.getState(stateIndex)))

    # Interpolate between states and check for validity
    for i in range(1, len(relevantStates)):
        # Check in between these points
        x, prevState = relevantStates[i - 1]
        nextStateIndex, nextState = relevantStates[i]
        hasObstacle = (not spaceInformation.checkMotion(prevState, nextState))
        if hasObstacle:
            # Return nextStateIndex, not i. In the example from docstring scenario 1, waypoint 4 refers to
            # i = 2 (2 away from B), but we want to return nextStateIndex = 4
            return nextStateIndex

    '''Uncomment to visualize the obstacles, relevant states, and all states
    import matplotlib.pyplot as plt
    ax = plt.gca()
    for obstacle in omplPath._ss.getStateValidityChecker().obstacles:
        obstacle.addPatch(ax)
    for s in omplPath.getSolutionPath().getStates():
        c = plt.Circle((s.getX(), s.getY()), radius=0.2)
        c.set_color('g')
        ax.add_patch(c)
    for s in relevantStates:
        c = plt.Circle((s.getX(), s.getY()), radius=0.1)
        c.set_color('b')
        ax.add_patch(c)

    c = plt.Circle((positionXY[0], positionXY[1]), radius=0.05)
    c.set_color('m')
    ax.add_patch(c)
    plt.show()
    plt.cla()
    '''

    # No obstacle on path found
    return -1


def plan(run_time, planner_type, wind_direction_degrees, dimensions, start_pos, goal_pos, obstacles, heading_degrees,
         state_sampler='',
         objective_type='sailing'):
    # Construct the robot state space in which we're planning
    space = ob.SE2StateSpace()

    # Create bounds on the position
    bounds = ob.RealVectorBounds(2)
    x_min, y_min, x_max, y_max = dimensions
    bounds.setLow(0, x_min)
    bounds.setLow(1, y_min)
    bounds.setHigh(0, x_max)
    bounds.setHigh(1, y_max)
    space.setBounds(bounds)

    # Use custom state sampler
    if len(state_sampler) > 0:
        if 'grid' in state_sampler.lower():
            space.setStateSamplerAllocator(ob.StateSamplerAllocator(GridStateSampler))
        else:
            print("WARNING: Unknown state_sampler = {}".format(state_sampler))

    # Define a simple setup class
    ss = og.SimpleSetup(space)

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()

    # Set resolution of state validity checking, which is fraction of space's extent (Default is 0.01)
    desiredResolution = VALIDITY_CHECKING_RESOLUTION_KM / ss.getStateSpace().getMaximumExtent()
    si.setStateValidityCheckingResolution(desiredResolution)

    # Set the objects used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(si, obstacles)
    ss.setStateValidityChecker(validity_checker)

    # Set our robot's starting state
    start = ob.State(space)
    start[0] = start_pos[0]
    start[1] = start_pos[1]

    # Set our robot's goal state
    goal = ob.State(space)
    goal[0] = goal_pos[0]
    goal[1] = goal_pos[1]

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal)

    # Create the optimization objective (helper function is simply a switch statement)
    objective = ph.allocate_objective(si, objective_type, wind_direction_degrees, heading_degrees)
    ss.setOptimizationObjective(objective)

    # Construct the optimal planner (helper function is simply a switch statement)
    optimizing_planner = allocatePlanner(si, planner_type)
    ss.setPlanner(optimizing_planner)

    # Attempt to solve the planning problem in the given runtime
    ss.solve(run_time)

    # Return the SimpleSetup object, which contains the solutionPath and spaceInformation
    # Must return ss, or else the object will be removed from memory
    return ss
