#!/usr/bin/env python

import sys
import os.path
import socket
import rospy
import libnmea_navsat_driver.driver
import tf

from htp_auto.msg import OdometryUTC

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
    pitch = euler[1]
    yaw = euler[2]
    for client in clients:
        try:
            client.send_position(utc_time, frame, x, y, z, zone)
            client.send_orientation(utc_time, rotation_frame, rotation_type, roll, pitch, yaw)
        except socket.error:
            rospy.logerr('Socket error: client could not send time/position information. Trying to reconnect to {0}'.format(client.address))
            try:
                client.reconnect()
            except socket.error, e:
                rospy.logerr('Error reconnecting: {0}'.format(e))

class SensorControlClient:
    '''Basic TCP client that allows client to send time, position and commands to sensors.'''
    
    def __init__(self, host, port):
        # SOCK_STREAM is the socket type to use for TCP sockets
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.address = (host, port)
        
    def connect(self):
        '''Connect to server.'''
        self.sock.connect(self.address)
    
    def reconnect(self):
        '''Create new socket (cannot reuse old one) and connect to it.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect()
    
    def close(self):
        '''Disconnect from server.'''
        self.sock.close()
        
    def is_connected(self):
        '''Return true if connected.  Check by trying to send data over connection.'''
        try:
            self.sock.send('?')
            return True
        except socket.error:
            return False
    
    def send_time(self, time):
        '''Send time over socket.'''
        self.sock.sendall("<t,{}>".format(time))
        
    def send_position(self, time, frame, x, y, z, zone=None):
        '''Send time and position in the specified frame.'''
        self.sock.sendall("<p,{},{},{},{},{},{}>".format(time, frame, x, y, z, zone))
        
    def send_orientation(self, time, frame, rotation_type, r1, r2, r3, r4=0):
        '''Send time and orientation in the specified frame.'''
        self.sock.sendall("<o,{},{},{},{},{},{},{}>".format(time, frame, rotation_type, r1, r2, r3, r4))
        
    def send_command_by_type(self, sensor_type, command):
        '''Send command to all sensors of specified type.'''
        self.sock.sendall("<ct,{},{}>".format(sensor_type, command))
        
    def send_command_by_name(self, sensor_name, command):
        '''Send command to all sensors with matching name (names aren't unique)'''
        self.sock.sendall("<cn,{},{}>".format(sensor_name, command))
        
    def send_command_by_id(self, sensor_id, command):
        '''Send command to sensor with matching ID. All ID's are unique.'''
        self.sock.sendall("<ci,{},{}>".format(sensor_id, command))

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
        
    clients = []
    for host in hosts:
        host_name = host[0]
        port = int(host[1])
        rospy.loginfo('Connecting to server at {0}:{1}'.format(host_name, port))
        client = SensorControlClient(host_name, port)
        try:
            client.connect()
        except socket.error, e:
            rospy.logerr('Error connecting to server: {0}'.format(e))
        clients.append(client)
        
    if clients is None:
        rospy.logwarn('No clients to connect to.  Exiting sensor control client.')
        sys.exit(0)
        
    rospy.Subscriber("gps_utc", OdometryUTC, odom_callback, queue_size = 50)
    
    rospy.spin()                               
