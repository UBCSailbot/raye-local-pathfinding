#! /usr/bin/env python
import argparse
import json
import urllib2

VAN_ISL_LAT = 48.5
VAN_ISL_LON = -125.0
MAUI_LAT = 21.0
MAUI_LON = -156.0

EXACTAIS_SHIPS_FN = 'exactais_ships.json'


def get_bounding_box_strings(bot_left_lat, bot_left_lon, top_right_lat, top_right_lon, dimension):
    '''Makes a `dimension`x`dimension` grid between the given bottom left and top right coordinates,
    and returns the corresponding bounding box dictionary.
        - Bounding box dictionary keys: '({column}, {row}): {bottom left latlon}-{top right latlon}'
        - Bounding box dictionary values: bottom left, top left, top right, botom right, then bottom left coordinates,
          where the coordinates are in the form '{longitude}%20{latitude}' and separated by '%20'
        - Input latitudes and longitudes must be floats
        - `dimension` must be an integer
    '''
    lon_diff = (top_right_lon - bot_left_lon) / dimension
    lat_diff = (top_right_lat - bot_left_lat) / dimension

    bounding_box_dict = {}
    curr_bot_lat = bot_left_lat
    for row in range(dimension):
        curr_top_lat = curr_bot_lat + lat_diff
        curr_left_lon = bot_left_lon
        for col in range(dimension):
            curr_right_lon = curr_left_lon + lon_diff
            key = '({},{}): ({},{})-({},{})'.format(col, row, curr_bot_lat, curr_left_lon, curr_top_lat, curr_right_lon)

            curr_bounding_box_list = [
                curr_left_lon, curr_bot_lat,
                curr_left_lon, curr_top_lat,
                curr_right_lon, curr_top_lat,
                curr_right_lon, curr_bot_lat,
                curr_left_lon, curr_bot_lat
            ]
            bounding_box_string = '%20'.join([str(coord) for coord in curr_bounding_box_list])

            bounding_box_dict[key] = bounding_box_string

            curr_left_lon = curr_right_lon
        curr_bot_lat = curr_top_lat

    return bounding_box_dict


def get_exactais_ships(token, bounding_box_string):
    url = 'https://services.exactearth.com/gws/wms?service=WFS&version=1.1.0&authKey=' + token + \
          '&request=GetFeature&typeName=exactAIS:LVI&outputFormat=json&' + \
          'filter=<Filter%20xmlns:gml="http://www.opengis.net/gml">' + \
          '<Intersects><PropertyName>position</PropertyName>' + \
          '<gml:Polygon%20xmlns:gml="http://www.opengis.net/gml"%20srsName="EPSG:4326">' + \
          '<gml:exterior><gml:LinearRing><gml:posList>' + bounding_box_string + \
          '</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></Intersects></Filter>'
    request = urllib2.Request(url)
    data = urllib2.urlopen(request)
    data = data.read()
    if data.startswith('{\n"errors"'):
        raise AssertionError('Queried server too often, wait for a couple minutes.\nData read from server:\n{}'
                             .format(data))

    data_json = json.loads(data)
    return data_json['features']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Saves relevant ExactAIS ship data to exactais_ships.json')
    parser.add_argument('token', help='ExactAIS token')
    args = parser.parse_args()

    bounding_box_dict = get_bounding_box_strings(MAUI_LAT, MAUI_LON, VAN_ISL_LAT, VAN_ISL_LON, dimension=5)

    exactais_ships = {}
    for key, bounding_box_string in bounding_box_dict.items():
        raw_ships_data = get_exactais_ships(args.token, bounding_box_string)
        exactais_ships[key] = raw_ships_data

    with open(EXACTAIS_SHIPS_FN, 'w') as f:
        json.dump(exactais_ships, f, indent=2)
