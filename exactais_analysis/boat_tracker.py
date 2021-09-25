#! /usr/bin/env python
import argparse
import json
import time
import urllib2

from ships_scraper import VAN_ISL_LAT, VAN_ISL_LON, MAUI_LAT, MAUI_LON, get_bounding_box_strings, get_exactais_ships

SECONDS_PER_MINUTE = 60


if __name__ == '__main__':
    # TODO: save bounding box that boat started from
    parser = argparse.ArgumentParser(description='Saves lists of speed and heading of a boat to json files,'
                                                 'polled every minute')
    parser.add_argument('token', help='ExactAIS token')
    parser.add_argument('time', type=int, help='Time in minutes (int) that the boat will be tracked for')
    parser.add_argument('bounding_box', help='Coordinate of bounding box to get boat from, i.e. 01 for (0,1)')
    args = parser.parse_args()

    bb_coord = '({},{})'.format(args.bounding_box[0], args.bounding_box[1])
    bounding_box_dict = get_bounding_box_strings(MAUI_LAT, MAUI_LON, VAN_ISL_LAT, VAN_ISL_LON, dimension=5)

    for key, bounding_box_string in bounding_box_dict.items():
        if key.startswith('{}: '.format(bb_coord)):
            initial_ships_data = get_exactais_ships(args.token, bounding_box_string)
            break

    tracked_boat_properties = initial_ships_data[0]['properties']  # track first boat in initial_ships_data
    tracked_boat_mmsi = tracked_boat_properties['mmsi']
    print('tracking boat with mmsi={} in bounding box {} for {} min'.format(tracked_boat_mmsi, bb_coord, args.time))

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

    print('finished tracking boat with mmsi={} at bounding box {}'.format(tracked_boat_mmsi, bb_coord))

    file_path = 'boat_tracker_data/bb_{}_mmsi_{}.json'.format(args.bounding_box, tracked_boat_mmsi)
    with open(file_path, 'w') as f:
        json.dump({'speeds': tb_speeds, 'headings': tb_headings}, f)

    print('wrote speeds and headings to {}'.format(file_path))
