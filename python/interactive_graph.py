import matplotlib.pyplot as plt
import pickle as pl
import os

# match with collision_checker.py
COLLISION_RADIUS_KM = 0.1
WARN_RADIUS_KM = 0.3


def plot_min_distances(folder_name):

    ABS_PATH_TO_THIS_FILE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
    PATH_TO_PKL = os.path.join(ABS_PATH_TO_THIS_FILE_DIRECTORY, "../output/", folder_name, "minDistanceInteractive.pkl")

    # Load figure from disk and display
    fig_handle = pl.load(open(PATH_TO_PKL, 'rb'))
    x, y = fig_handle.axes[0].lines[0].get_data()

    # Getting warning times
    i = 1  # i = 0 is y intercept, not wanted
    warn_start_times = list()
    while i < len(y):
        if y[i] < WARN_RADIUS_KM:
            warn_start_times.append(x[i])
            while i + 1 < len(y) and y[i + 1] >= y[i]:
                i += 1
        i += 1

    print(warn_start_times)

    # Sequentially looking around each warning start time
    for warning in warn_start_times:
        pl.load(open(PATH_TO_PKL, 'rb'))
        print ("Warning around {} seconds" .format(warning))
        plt.ylim(0, 1)
        plt.xlim(warning - 10, warning + 10)
        plt.show()


plot_min_distances("Nov-12-2020_18-30-40")
