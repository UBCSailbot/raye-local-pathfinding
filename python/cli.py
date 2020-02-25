import argparse
import math

from ompl import util as ou

from updated_geometric_planner import plan, Obstacle
from plotting import plot_path

def parse_obstacle(s):
    try:
        x, y, radius = map(float, s.split(','))
        return Obstacle(x, y, radius)
    except Exception:
        raise argparse.ArgumentTypeError("Obstacles must be x,y,radius")

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Sailbot motion planning CLI.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=3.0, help=
    '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRTstar', \
        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar'], \
         help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='WeightedLengthAndClearanceCombo',
                        choices=['PathClearance', 'PathLength', 'ThresholdPathLength',
                                 'WeightedLengthAndClearanceCombo'],
                        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-i', '--info', type=int, default=2, choices=[0, 1, 2],
                        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
                             ' Defaults to WARN.')
    parser.add_argument('-w', '--windDirection', type=lambda x: int(x), default=-135,
                        help='(Optional) Wind direction in degrees (direction the wind is going)')

    parser.add_argument('-d', '--dimensions', nargs=4, type=int, default=[0, 0, 10, 10],
                        help='(Optional) dimensions of the space')
    parser.add_argument('-g', '--goal', nargs=2, type=int, default=[10,10],
                        help='(Optional) planner goal')
    parser.add_argument('-s', '--start', nargs=2, type=int, default=[0,0],
                        help='(Optional) planner start')
    parser.add_argument('-ob', '--obstacles', nargs='+', type=parse_obstacle, default=[parse_obstacle("2.5,2.5,1")],
                        help='(Optional) dimensions of the space')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Find the dimensions of the space
    max_dist = max(args.goal)
    args.dimensions = [
            -abs(max_dist),
            -abs(max_dist),
            abs(max_dist),
            abs(max_dist),
            ]

    # Solve the planning problem multiple times and find the best one
    solutions = []
    numRuns = 4
    for i in range(numRuns):
        solutions.append(plan(args.runtime, args.planner, args.objective, args.windDirection, args.dimensions, args.start, args.goal,
         args.obstacles))
    solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())

    # Print and plot the solution path
    print("Solution Path Before Interpolation:")
    for state in solution.getSolutionPath().getStates():
        print("{}, {}, {}".format(state.getX(), state.getY(), state.getYaw()))
    print("******************")
    plot_path(solution.getSolutionPath().printAsMatrix(), args.dimensions, args.obstacles)

    # Interpolate the path, which ensures that the path has >= 10 waypoints
    # Reprint and plot
    solution.getSolutionPath().interpolate(10)
    print("Solution Path After Interpolation:")
    for state in solution.getSolutionPath().getStates():
        print("{}, {}, {}".format(state.getX(), state.getY(), state.getYaw()))
    print("******************")
    plot_path(solution.getSolutionPath().printAsMatrix(), args.dimensions, args.obstacles)

