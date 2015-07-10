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
from __builtin__ import True

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
    last_utc_time = 0
    for topic, msg, t in bag.read_messages(topics=[position_topic]):
        try:
            position = msg.pose.pose.position
        except AttributeError:
            position = msg.odom.pose.pose.position
            utc_time = msg.time
            if utc_time - last_utc_time < 0:
                print 'WARNING MESSAGES NOT SORTED IN ORDER BY UTC TIME'
            last_utc_time = utc_time
        
        x = position.x
        y = position.y
        z = position.z
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
        home_positions.append([lat, lon, alt, source])
        
    bag.close()
    
    return home_positions

def distanceBetweenPositionsXY(p1, p2):
    '''Returns magnitude of distance between two points.'''
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx*dx + dy*dy)

def simplifyPositionsMinDistance(positions, min_distance):
    '''Return new list of position where the spacing between any two consecutive points is no less than specified distance in XY plane.'''
    simplified_positions = []
    
    for i, position in enumerate(positions):
        if i == 0:
            simplified_positions.append(position)
            continue
        last_kept_position = simplified_positions[-1]
        if distanceBetweenPositionsXY(position, last_kept_position) > min_distance:
            simplified_positions.append(position)
        
    return simplified_positions
            
def simplifyPositionsApprox(positions, epsilon):
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
    
    if len(home_positions) == 0:
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

def convertBagToMissionFile(bag_path, output_path, simplify_type, position_topic, home_topic, epsilon, min_distance, radius, go):
    '''Main function to simplify logged position data and write as waypoints to mission file.'''
    positions = extractPositionsFromBag(bag_path, position_topic)
    
    if len(positions) == 0:
        print '\nNo position messages found in bag using topic \'{}\''.format(position_topic)
        return False
    
    print '\nExtracted {} position messages.'.format(len(positions))
            
    if simplify_type == 'approx':
        positions = simplifyPositionsApprox(positions, epsilon)
    elif simplify_type == 'distance':
        positions = simplifyPositionsMinDistance(positions, min_distance)
    else:
        print 'Warning: invalid method ' + simplify_type
    
    print 'Simplified down to {} positions using {} method.'.format(len(positions), simplify_type)
            
    mission_items = convertPositionsToMissionItems(positions, radius, go)
    
    if len(mission_items) == 0:
        print '\nError during conversion.'
        return False
    
    print 'Converted all positions to waypoints.'
    
    home_positions = extractHomePositionsFromBag(bag_path, home_topic)

    home_position = chooseHomePosition(home_positions)
    if home_position is not None:
        set_home_command = convertHomeToSetCommand(home_position)
        mission_items.insert(0, set_home_command)
    
    writeMissionItemsToFile(mission_items, output_path)
    
    print 'Wrote mission file: {0}'.format(output_path)
    print ''
    
    return True # Successful

if __name__ == '__main__':
    
    simplify_types = ['approx', 'distance']
    default_simplify_type = simplify_types[0]
    default_epsilon = .1
    default_distance = 1 # meters
    default_radius = .5 # meters
    default_home_topic = '/home'
    default_pos_topic = '/gps'
    default_go = 'false'
    
    parser = argparse.ArgumentParser(description='Convert position messages in bag file to mission file.')
    parser.add_argument('bag_file', help='path to bag file to convert')
    parser.add_argument('output_file', help='path to mission file to create')
    parser.add_argument('-s', dest='simplify_type', default=default_simplify_type, help='How to simplify points. Options are {}. Approx is a deviation from path and distance deletes points to maintain spacing. Default is {}'.format(simplify_types, default_simplify_type))
    parser.add_argument('-p', dest='position_topic', default=default_pos_topic, help='Name of topic to extract position from. Default {}'.format(default_pos_topic))
    parser.add_argument('-t', dest='home_topic', default=default_home_topic, help='Name of topic to extract home position from. Default {}'.format(default_home_topic))
    parser.add_argument('-e', dest='epsilon', default=default_epsilon, help='Split threshold in meters. Smaller = more waypoints. Must be greater than 0. Default {}'.format(default_epsilon))
    parser.add_argument('-d', dest='min_distance', default=default_distance, help='Minimum waypoint spacing when using distance type. Default {}'.format(default_distance))
    parser.add_argument('-a', dest='radius', default=default_radius, help='Acceptance radius of every waypoint. Default {}'.format(default_radius))
    parser.add_argument('-g', dest='go', default=default_go, help='If \'true\' then will only stop at first and last waypoints. Default {}'.format(default_go))
    args = parser.parse_args()
    
    bag_path = args.bag_file
    output_path = args.output_file
    simplify_type = args.simplify_type.lower()
    position_topic = args.position_topic
    home_topic = args.home_topic
    epsilon = float(args.epsilon)
    min_distance = float(args.min_distance)
    radius = float(args.radius)
    go = args.go.lower() == 'true'; 
    
    if epsilon <= 0:
        print "\nError: epsilon must be greater than zero.\n"
        parser.print_help()
        sys.exit(1)
        
    if simplify_type not in simplify_types:
        print '\nError: simplify type {} not one of options {}'.format(simplify_type, simplify_types)
        sys.exit(1)
        
    success = convertBagToMissionFile(bag_path, output_path, simplify_type, position_topic, home_topic, epsilon, min_distance, radius, go)
        
    if success: 
        print 'Success\n'
        sys.exit(0)
    else:
        sys.exit(1)
    
