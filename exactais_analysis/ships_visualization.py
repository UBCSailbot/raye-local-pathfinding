#! /usr/bin/env python
import argparse
import json
import matplotlib.pyplot as plt

from ships_scraper import EXACTAIS_SHIPS_PATH

HEATMAP_PATH = 'ships_data/heatmap.txt'


def create_heatmap(exactais_ships):
    '''Create heatmap text file compatible with http://heatmapper.ca/geocoordinate/'''
    f = open(HEATMAP_PATH, 'w')
    f.write('{}\t{}\n'.format('Longitude', 'Latitude'))
    for bounding_box_ships in exactais_ships.values():
        for ship in bounding_box_ships:
            ship_properties = ship['properties']
            f.write('{}\t{}\n'.format(ship_properties['longitude'], ship_properties['latitude']))
    f.close()


def create_histograms(exactais_ships):
    '''Create histograms of key attributes of `exactais_ships` using matplotlib'''
    ships_speedKnots = []
    ships_headingDegrees = []
    ships_length = []
    ships_width = []
    for bounding_box_ships in exactais_ships.values():
        for ship in bounding_box_ships:
            ship_properties = ship['properties']
            ships_speedKnots.append(ship_properties['sog'])
            ships_headingDegrees.append(ship_properties['cog'])
            if ship_properties['length'] is not None and ship_properties['length'] != 0 and \
                    ship_properties['width'] is not None and ship_properties['width'] != 0:
                ships_length.append(ship_properties['length'])
                ships_width.append(ship_properties['width'])

    # Print all ship's (length, width), used for manually checking their values
    # print([(ships_length[i], ships_width[i]) for i in range(len(ships_length))])

    plot_histogram(data=ships_speedKnots, header='Speed (knots)')
    plot_histogram(data=ships_headingDegrees, header='Heading (degrees)')
    plot_histogram(data=ships_length, header='Length (metres)')
    plot_histogram(data=ships_width, header='Width (metres)')


def plot_histogram(data, header, bins=10):
    '''Configure and plot a matplotlib histogram'''
    plt.figure(num='ExactAIS Ship Data Histograms')
    plt.title(header)
    plt.ylabel('# Ships')
    plt.grid(alpha=0.5)
    plt.hist(data, bins=bins)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualizes ExactAIS ship data from ships_in_bounding_boxes.json')
    parser.add_argument('--heatmap', action='store_true', help='create heatmap')
    parser.add_argument('--histograms', action='store_true', help='create histograms')
    args = parser.parse_args()

    with open(EXACTAIS_SHIPS_PATH, 'r') as f:
        exactais_ships = json.load(f)

    if args.heatmap:
        create_heatmap(exactais_ships)
    if args.histograms:
        create_histograms(exactais_ships)
