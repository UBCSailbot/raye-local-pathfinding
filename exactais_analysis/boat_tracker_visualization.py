#! /usr/bin/env python
import argparse
import json
import matplotlib.pyplot as plt


def create_plots(mmsi, bounding_box, tb_speeds, tb_headings):
    '''Configure and plot matplotlib plots'''
    plots_name = 'bb_{}_mmsi_{}'.format(bounding_box, mmsi)
    fig, (speed_plot, heading_plot) = plt.subplots(nrows=1, ncols=2, num=plots_name, figsize=(13, 6))
    suptitle = 'Plots for Boat {} in Bounding Box ({},{})'.format(mmsi, bounding_box[0], bounding_box[1])
    fig.suptitle(suptitle)
    time_coords = range(len(tb_headings))

    speed_plot.set_title('Speed (Knots) vs Time (Minutes)')
    speed_plot.plot(time_coords, tb_speeds)

    heading_plot.set_title('Heading (Degrees) vs Time (Minutes)')
    heading_plot.plot(time_coords, tb_headings)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot the speed and headings lists from exactais_ship_tracker.py')
    parser.add_argument('bounding_box', help='Coordinate of bounding box to get boat from, i.e. 01 for (0,1)')
    parser.add_argument('mmsi', type=int, help='mmsi (int) of the ship that was tracked')
    args = parser.parse_args()

    with open('boat_tracker_data/bb_{}_mmsi_{}.json'.format(args.bounding_box, args.mmsi), 'r') as f:
        data = json.load(f)

    tb_speeds = data['speeds']
    tb_headings = data['headings']
    create_plots(args.mmsi, args.bounding_box, tb_speeds, tb_headings)
