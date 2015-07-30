#!/usr/bin/env python

import sys
import os.path
import socket
import rospy
import libnmea_navsat_driver.driver
import tf

from htp_auto.msg import OdometryUTC

debug_count_ = 0

def odom_callback(odom):
    '''Subscriber callback for odometery message. Pass along data to each sensor client.'''
    pose = odom.odom.pose.pose
    # Report UTM coordinates instead of local ENU so can geo-reference sensors to geoid.
    frame = 'UTM'
    x = odom.easting #pose.position.x 
    y = odom.northing #pose.position.y
    z = odom.altitude #pose.position.z
    zone = odom.utm_zone
    utc_time = odom.time
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    rotation_frame = 'ENU'
    rotation_type = 'FixedXYZ'
    roll = euler[0]
    pitch = 'nan' # euler[1] pitch isn't being measured so don't want to store it as 0 in database.
    yaw = euler[2]
    
    # Calculate amount of time that has elapsed since first reading in GPS message and now.
    processing_duration = rospy.get_rostime() - odom.odom.header.stamp
    processing_seconds = processing_duration.to_sec()
    
    debug_count_ += 1
    if debug_count_ % 20 == 0:
        rospy.loginfo('Merge: {}  Processing: {}'.format(odom.merge_time_diff, processing_seconds))
    
    for client in clients:
        try:
            client.send_position(utc_time, frame, x, y, z, zone)
            client.send_orientation(utc_time, rotation_frame, rotation_type, roll, pitch, yaw)
        except socket.error:
            rospy.logerr('Socket error - Address: {} Message: {}'.format(client.address, e))

class SensorControlClient:
    '''Basic UDP client that allows client to send time, position and commands to sensors.
    
        There is no connect() call since UDP has no connections.'''
    
    def __init__(self, host, port):
        # SOCK_DGRAM is the socket type to use for UDP sockets
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)
    
    def send_time(self, time):
        '''Send time over socket.'''
        self.sock.sendto("t,{}\n".format(time), self.address)

    def send_position(self, time, frame, x, y, z, zone=None):
        '''Send time and position in the specified frame.'''
        self.sock.sendto("p,{},{},{},{},{},{}\n".format(time, frame, x, y, z, zone), self.address)
        
    def send_orientation(self, time, frame, rotation_type, r1, r2, r3, r4=0):
        '''Send time and orientation in the specified frame.'''
        self.sock.sendto("o,{},{},{},{},{},{},{}\n".format(time, frame, rotation_type, r1, r2, r3, r4), self.address)
        
    def send_command_by_type(self, sensor_type, command):
        '''Send command to all sensors of specified type.'''
        self.sock.sendto("ct,{},{}\n".format(sensor_type, command), self.address)
        
    def send_command_by_name(self, sensor_name, command):
        '''Send command to all sensors with matching name (names aren't unique)'''
        self.sock.sendto("cn,{},{}\n".format(sensor_name, command), self.address)
        
    def send_command_by_id(self, sensor_id, command):
        '''Send command to sensor with matching ID. All ID's are unique.'''
        self.sock.sendto("ci,{},{}\n".format(sensor_id, command), self.address)

if __name__ == '__main__':
    
    rospy.init_node('sensor_control_client')
    
    default_server_host = socket.gethostname()
    default_server_port = 5000
    default_host = '{0} {1}'.format(default_server_host, default_server_port)

    # Determine if host is file path or a list. 
    hosts = rospy.get_param('~sensor_hosts', "none")
    if os.path.exists(hosts):
        # Replace hosts variable with file contents to mimic passing in on command line.
        with open(hosts) as hosts_file:
            hosts = hosts_file.read()

    # Switch out any reference to hostname with actual host name.
    hosts = hosts.replace('hostname', socket.gethostname())

    # Split hosts up into list.
    hosts = hosts.replace('\n',' ').replace('\t',' ').replace(',', ' ').split()

    # Check if we need to also use default host.
    include_default_host = False
    if 'default' in hosts:
        include_default_host = True
        hosts.remove('default')

    # Pair every two elements in a tuple. Ignores an extra element at end.
    hosts = zip(hosts,hosts[1:])[::2]
    
    if include_default_host:
        # Now that things are paired insert default host at beginning of list.
        hosts.insert(0, (default_server_host, default_server_port))
        
    # Connect a client to each host.
    clients = []
    for host in hosts:
        host_name = host[0]
        port = int(host[1])
        rospy.loginfo('Creating client for {0}:{1}'.format(host_name, port))
        client = SensorControlClient(host_name, port)
        clients.append(client)
        
    if clients is None:
        rospy.logwarn('No clients to connect to.  Exiting sensor control client.')
        sys.exit(0)
        
    # Keep buffer size at 1 to avoid buffering old messages.
    rospy.Subscriber("gps_utc", OdometryUTC, odom_callback, queue_size = 1)
    
    rospy.spin()                               
