#! /usr/bin/env python
'''
@author: Kyle McGahee

@summary: Converts bag file CSV file(s) and can convert quaternions to RPY.
'''
import sys
import os
import math
import argparse
import rosbag
import csv
import tf

'''
def getBagTopicsAndTypes(bag):
    print [method for method in dir(bag) if callable(getattr(bag, method))]
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])
    return zip(topics, types)
'''

def extractMessagesFromBag(bag, topic):
    '''Returns list of all messages in specified topic from bag.'''
    
    if topic != '/gps_utc':
        print "Only supports /gps_utc topic right now. Exiting."
        sys.exit(1)
    
    positions = []
    orientations = []
    for top, msg, timestamp in bag.read_messages([topic]):
        utc_time = msg.time
        pose = msg.odom.pose.pose
        x = msg.easting #pose.position.x 
        y = msg.northing #pose.position.y
        z = msg.altitude #pose.position.z
        zone = msg.utm_zone
        
        positions.append((utc_time, x, y, z, zone))
        
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        roll = euler[0]
        pitch = 'nan' # euler[1] pitch isn't being measured so don't want to store it as 0 in database.
        yaw = euler[2]
            
        orientations.append((utc_time, roll, pitch, yaw))
    
    return positions, orientations

def writeToCSVFile(filepath, buffer):
    # Make sure file is open so we can write to it.
    with open(filepath, 'wb') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerows(buffer)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Convert position messages in bag file to mission file.')
    parser.add_argument('bag_file', help='path to bag file to convert')
    parser.add_argument('topics', help='list of topics separated by commas to extract to their own csv file. If \'all\' then will extract all topics.')
    args = parser.parse_args()
    
    bag_filepath = args.bag_file
    topics = args.topics.split(',')
    quat_topic = args.topics
    
    if not os.path.exists(bag_filepath):
        print "Bag path doesn't exist {}".format(bag_filepath)
        sys.exit(1)
        
    bag = rosbag.Bag(bag_filepath)
    
    '''
    topics_in_bag = getBagTopicsAndTypes(bag)
    print "Found {} topics in bag.".format(len(topics_in_bag))
    for (topic_name, topic_type) in topics_in_bag:
        print "Topic: {} Type: {}".format(topic_name, topic_type)
                
    if topics.lower() == 'all':
        topics = [topic_name for (topic_name, topic_type) in topics_in_bag]
     '''
    
    bag_dir, bag_filename = os.path.split(bag_filepath)
    bag_basename = os.path.splitext(bag_filename)[0]
    
    out_dir = os.path.join(bag_dir, bag_basename)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    for topic in topics:
        
        positions, orientations = extractMessagesFromBag(bag, topic)
        
        if len(positions) == 0:
            print "No messages found for topic {}".format(topic)
            continue
        
        out_filename = "{}_positions.csv".format(bag_basename, topic.replace('/',''))
        out_filepath = os.path.join(out_dir, out_filename)
        writeToCSVFile(out_filepath, positions)
        
        out_filename = "{}_orientations.csv".format(bag_basename, topic.replace('/',''))
        out_filepath = os.path.join(out_dir, out_filename)
        writeToCSVFile(out_filepath, orientations)
 
    '''
    for topic in topics:
        
        messages = extractMessagesFromBag(bag, topic)
        
        if len(messages) == 0:
            print "No messages found for topic {}".format(topic)
            continue
        
        out_filename = "{}_{}.csv".format(bag_basename, topic.replace('/',''))
        out_filepath = os.path.join(out_dir, out_filename)
        
        writeToCSVFile(out_filepath, messages)
        print "Wrote topic {} to {}".format(topic, out_filename)
    ''' 
        
        
    
