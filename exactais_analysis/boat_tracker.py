#! /usr/bin/env python
import argparse
import json
import time
import urllib2

from ships_scraper import VAN_ISL_LAT, VAN_ISL_LON, MAUI_LAT, MAUI_LON, get_bounding_box_strings, get_exactais_ships

SECONDS_PER_MINUTE = 60


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Saves lists of speed and heading of a boat to json files,'
                                                 'polled every minute')
    parser.add_argument('token', help='ExactAIS token')
    parser.add_argument('index', type=int,
                        help='Index (int) of the ship to track from the ships list in between Vancouver and Maui')
    parser.add_argument('time', type=int, help='Time in minutes (int) that the boat will be tracked for')
    args = parser.parse_args()

    bounding_box_dict = get_bounding_box_strings(MAUI_LAT, MAUI_LON, VAN_ISL_LAT, VAN_ISL_LON, dimension=5)

    for key, bounding_box_string in bounding_box_dict.items():
        if key.startswith('(2,2): '):  # (2,2) is the middle square in 5x5 0-indexed grid
            initial_ships_data = get_exactais_ships(args.token, bounding_box_string)
            break

    tracked_boat_properties = initial_ships_data[args.index]['properties']
    tracked_boat_mmsi = tracked_boat_properties['mmsi']
    print('start tracking boat with mmsi={}'.format(tracked_boat_mmsi))

    tb_speeds = []
    tb_headings = []
    for i in range(int(args.time)):
        tb_speeds.append(tracked_boat_properties['sog'])
        tb_headings.append(tracked_boat_properties['cog'])

        tb_lon = tracked_boat_properties['longitude']
        tb_lat = tracked_boat_properties['latitude']
        bounding_box_dict = get_bounding_box_strings(tb_lat-0.5, tb_lon-0.5, tb_lat+0.5, tb_lon+0.5, dimension=1)

        try:
            ships_data = get_exactais_ships(args.token, bounding_box_dict.values()[0])
        except urllib2.URLError:
            print('caught URLError when trying to retrieve ExactAIS ships at t={}, sleeping for 1 min'.format(i))
            continue
        for ship in ships_data:
            if ship['properties']['mmsi'] == tracked_boat_mmsi:
                tracked_boat_properties = ship['properties']
                break

        print('found tracked boat at t={} min, sleeping for 1 min'.format(i))
        time.sleep(SECONDS_PER_MINUTE)

    print('finished tracking boat with mmsi={}'.format(tracked_boat_mmsi))

    file_path = 'boat_tracker_data/boat_{}.json'.format(tracked_boat_mmsi)
    with open(file_path, 'w') as f:
        json.dump({'speeds': tb_speeds, 'headings': tb_headings}, f)

    print('wrote speeds and headings to {}'.format(file_path))
