#! /usr/bin/env python
'''
@author: Kyle McGahee

@summary: Extracts position measurements from specified bag file, simplifies points in XY
          plane using split and merge algorithm and then writes results to mission file
          as waypoints.
'''
import sys
import os
import math
import argparse
import rosbag
from nmea_parser import parse_nmea_sentence

class TimeStampedPosition(object):

    def __init__(self, time, x, y, z):
        self.time = time
        self.x = x 
        self.y = y
        self.z = z
        
    @property
    def position(self):
        return (x, y, z)

# ------------------------------------------------------
# Helper classes/functions for split and merge algorithm
# ------------------------------------------------------
class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
def shortestDistanceToSegment(p, v, w):
    '''Return minimum distance between line segment vw and point p'''
    return math.sqrt(distToSegmentSquared(p, v, w))
  
def distToSegmentSquared(p, v, w):
    '''Return squared minimum distance between line segment vw and point p'''
    length_squared = distance_squared(v, w) # i.e. |w-v|^2
    
    if length_squared == 0:
        # v == w case
        return distance_squared(p, v)
    
    # Consider the line extending the segment, parameterized as v + t (w - v).
    # We find projection of point p onto the line. 
    # It falls where t = [(p-v) . (w-v)] / |w-v|^2
    t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / length_squared;
    if t < 0:
        # Beyond the 'v' end of the segment
        return distance_squared(p, v)
    if t > 1:
        # Beyond the 'w' end of the segment
        return distance_squared(p, w)
    
    # Projection falls on the segment
    projection = Point2D(v.x + t * (w.x - v.x), v.y + t * (w.y - v.y))
    return distance_squared(p, projection);
                    
def square(x):
    return x * x

def distance_squared(v, w):
    return square(v.x - w.x) + square(v.y - w.y)

def extractPositionsFromBag(bag_file_path, position_topic):
    '''Returns XYZ positions from specified topic where each pos. element is a list'''
    bag = rosbag.Bag(bag_file_path)

    positions = []    
    for topic, msg, t in bag.read_messages(topics=[position_topic]):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        positions.append([x, y, z])
        
    bag.close()
    
    return positions

def extractHomePositionsFromBag(bag_file_path, home_topic):
    '''Returns home positions from specified topic where each home element is a list'''
    bag = rosbag.Bag(bag_file_path)

    home_positions = []    
    for topic, msg, t in bag.read_messages(topics=[home_topic]):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        source = msg.source 
        home_positions.append((lat, lon, alt, source))
        
    bag.close()
    
    return home_positions

def extractLLAFromNmeaLog(log_file_path):
    '''Returns lat/long/altitude positions from specified NMEA log file.'''
    positions = []    
    with open(log_file_path) as log_file:
        for line in log_file.readlines():
            nmea_string = line.strip()

            if not check_nmea_checksum(nmea_string):
                print "Received a sentence with an invalid checksum. Sentence was: {}".format(nmea_string)
                continue
            
            parsed_sentence = parse_nmea_sentence(nmea_string)
            if not parsed_sentence:
                print "Failed to parse NMEA sentence {}".format(nmea_string)
                continue
            
            if 'GGA' in parsed_sentence:
                data = parsed_sentence['GGA']
                altitude = data['altitude'] # Altitude is above ellipsoid
            elif 'GGK' in parsed_sentence:
                data = parsed_sentence['GGK']
                # Strip off EHT prefix and convert to float
                try:
                    altitude = data['height_above_ellipsoid']
                    altitude = float(altitude[3:])
                except ValueError:
                    altitude = 0
            else:
                continue # not a supported position message 

            # common between position message types
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude 
            utc_time = data['utc_time']
            if math.isnan(utc_time):
                print 'Invalid UTC time: {}'.format(utc_time)
                continue
                
            positions.append(TimeStampedPosition(utc_time, latitude, longitude, altitude))
     
    return positions
    
def simplifyPositions(positions, epsilon):
    '''Uses first two elements of positions to run split and merge. 
        Returns same format as input positions but simplified.  
        Z component is set to zero.'''
    
    # Create 2D position using just XY component.
    points = [Point2D(p[0],p[1]) for p in positions]

    points = splitAndMerge(points, epsilon)
    
    # Convert back into same XYZ list format that was passed in.
    # Just set Z to 0 since we're not using it.
    simplified_positions = []
    for point in points:
        simplified_positions.append([point.x, point.y, 0])

    return simplified_positions

def splitAndMerge(points, epsilon):
    '''Returns simplified version of input point list.  Epsilon is the distance
       threshold that determines whether a segment is simplified or not.  A
       higher epsilon will lead to more simplification.'''
    
    # Find the point with the maximum distance from the start and end points.
    dist_max = 0
    index = 0 # Index of point with max distance.
    end_index = len(points) - 1
    for i in range(1, end_index - 1):
        dist = shortestDistanceToSegment(points[i], points[1], points[end_index])
        if dist > dist_max:
            index = i
            dist_max = dist
            
    # If max distance is greater than epsilon then this point needs to stay and
    # then recursively split and simplify the rest of the points.
    if dist_max > epsilon:
        # Recursive calls.  
        # First call ranges from start to index point (inclusive)
        # Second call ranges from index point to end (both inclusive)
        results1 = splitAndMerge(points[:index+1], epsilon)
        results2 = splitAndMerge(points[index:], epsilon)
 
        # Combine the two results, but first delete last element in first results
        # so it doesn't get counted twice.
        results1 = results1[:-1]
        result_list = results1 + results2
    else:
        # We can approximate the rest of the points with just the start and end.
        result_list = (points[0], points[-1])
    
    return result_list

def convertPositionsToMissionItems(positions, radius, go):
    '''Returns list of waypoints corresponding to positions.
        Every waypoint is setup for the robot to stop there.'''
    type = 0
    frame = 0
    stop = 1.0 # True
    
    if go:
        print 'Continuing through waypoints.'
    else:
        print 'Stopping at each waypoint.'
    
    mission_items = []
    for i, message in enumerate(positions):
        x = message[0]
        y = message[1]
        z = message[2]
        
        # If 'go' is true then only stop at first and last waypoints,
        # otherwise stop at all of them.
        first_or_last = (i == 0) or (i == len(positions)-1)
        if go and not first_or_last:
            stop = 0.0 # False
        else:
            stop = 1.0 # True
        
        mission_items.append([type, frame, x, y, z, radius, stop, 0, 0])
    
    return mission_items

def chooseHomePosition(home_positions):
    
    if home_positsions is None or len(home_positions) == 0:
        print '\nWarning: no home position found to include in mission.'
        return None

    print '\nHome locations found in bag file:'
    for i, position in enumerate(home_positions):
        print "Index {0}: {1}".format(i, position)

    valid_index = False
    while not valid_index:    
        index = raw_input('Enter index of home position to use for mission. -1 for none.  ')
        try:
            index = int(index)
            if index >= -1 and index < len(home_positions):
                valid_index = True
            
        except ValueError:
            print 'Non-integer input'

    if index == -1:
        return None
    else:
        return home_positions[index]

def convertHomeToSetCommand(home_position):
    
    type = 1
    x = home_position[0]
    y = home_position[1]
    z = home_position[2]
        
    set_home_command = [type, 0, x, y, z, 0, 0, 0, 0]
    return set_home_command

def writeMissionItemsToFile(mission_items, output_file_path):
    '''Creates (or overwrites) specified file and writes each item on a new CSV line.'''
    directory = os.path.dirname(output_file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    outfile = open(output_file_path, 'w+')

    for item in mission_items:
        for i, field in enumerate(item):            
            outfile.write(str(field))
            if i < (len(item) - 1):
                outfile.write(',')
        outfile.write('\n')
    
    outfile.close()

    return

def convertLogs(log_paths):
    '''Main function to simplify logged position data and write as waypoints to mission file.'''
    all_positions = []
    for log_path in log_paths:
        print 'Processing file: ' + os.path.splitext(log_path)[0]
        lla_positions = extractLLAFromNmeaLog(log_path)
 
        if len(positions) == 0:
            print 'No positions found in file.'''
        else:
            print 'Extracted {} positions.'.format(len(positions))
            all_positions.extend(positions)
    
    if len(all_positions) == 0:
        print 'No positions to process.'
        return None, None
    
    if len(log_paths) > 1:
        print 'Extracted {} positions from {} files.'.format(len(all_positions), len(log_paths)) 
        
    print 'Sorting positions by time stamp.'''
    lla_positions = sorted(lla_positions, key=lambda item: item.time)
    
    home_lat, home_lon, home_alt = lla_positions[0].position
    home_position = [(home_lat, home_lon, home_alt, 'First Item')]

    
    
    return positions, home_position

def convertBag(bag_path, position_topic, home_topic):
    '''Main function to simplify logged position data and write as waypoints to mission file.'''
    print 'Processing ' + bag_path
    
    positions = extractPositionsFromBag(bag_path, position_topic)
    
    if len(positions) == 0:
        print '\nNo position messages found in bag using topic \'{}\''.format(position_topic)
        return None, None
   
    home_positions = extractHomePositionsFromBag(bag_path, home_topic)

    return positions, home_positions
    
def getLogFiles(input):
    '''Returns list of filepaths to valid log files.  'input' argument is either a log filepath or a directory containing logs.'''
    log_filepaths = []
    extensions = ['bag', 'csv', 'log', 'cap']
    if os.path.isdir(input):
        for fname in os.listdir(input):
            extension = os.path.splitext(fname)[1][1:]
            if extension.lower() in extensions:
                if extension == 'bag':
                    print 'Skipping file {} since its a bag.  Only logs can be processed in groups.'.format(fname)
                else:
                    log_filenames.append(os.path.join(input, fname))
            else:
                print 'Skipping file {} due to unsupported extension'.format(fname)
    elif os.path.exits(input): # must be a valid file path
        extension = os.path.splitext(fname)[1][1:]
        if extension in extensions:
            log_filepaths.append(input)
        else:
            print 'File {} exists, but does not have one of the supported extensions {}'.format(input, extensions)
    else:
        print '{} does not exists.'
        
    return log_filepaths
     
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Convert position messages in log or bag file(s) to mission file.')
    parser.add_argument('input', help='either path to log/bag file to convert or if a directory then uses all logs/bags found inside.')
    parser.add_argument('output_file', help='path to mission file to create')
    parser.add_argument('-p', dest='position_topic', default='/robot_pose_ekf/odom_combined', help='Name of topic to extract position from.')
    parser.add_argument('-t', dest='home_topic', default='/home', help='Name of topic to extract home position from.')
    parser.add_argument('-e', dest='epsilon', default=.2, help='Split threshold in meters. Smaller = more waypoints. Must be greater than 0')
    parser.add_argument('-a', dest='radius', default=.5, help='Acceptance radius of every waypoint.')
    parser.add_argument('-g', dest='go', default='false', help='If \'true\' then will only stop at first and last waypoints')
    args = parser.parse_args()
    
    input = args.input
    output_path = args.output_file
    position_topic = args.position_topic
    home_topic = args.home_topic
    epsilon = float(args.epsilon)
    radius = float(args.radius)
    go = args.go.lower() == 'true'; 
    
    if epsilon <= 0:
        print "\nError: epsilon must be greater than zero.\n"
        parser.print_help()
        sys.exit(1)
        
    log_filepaths = getLogFiles(input)
        
    if len(log_filepaths) == 0:
        print 'No files to process.  Exiting.'
        sys.exit(1)
                   
    if os.path.splitext(log_filepaths[0])[1][1:] == 'bag':
        bag_filepath = log_filepaths[0]
        positions, home_positions = convertBag(bag_filepath, position_topic, home_topic)
    else:
        positions, home_positions = convertLogs(log_filepaths)
    
    if positions is None or len(positions) == 0:
        sys.exit(1)
        
    positions = simplifyPositions(positions, epsilon)
    
    print 'Simplified down to {} positions.'.format(len(positions))
            
    mission_items = convertPositionsToMissionItems(positions, radius, go)
    
    if len(mission_items) == 0:
        print '\nError during conversion.'
        return sys.exit(1)
    
    print 'Converted all positions to waypoints.'

    home_position = chooseHomePosition(home_positions)
    if home_position is not None:
        set_home_command = convertHomeToSetCommand(home_position)
        mission_items.insert(0, set_home_command)
        
    writeMissionItemsToFile(mission_items, output_path)
    
    print 'Wrote mission file: {}'.format(output_path)
    print ''
        
    if success: 
        print 'Success\n'
