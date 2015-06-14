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
    x = pose.position.x 
    y = pose.position.y
    z = pose.position.z
    utc_time = odom.time
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    for client in clients:
        try:
            client.send_time_position_and_orientation(utc_time, x, y, z, roll, pitch, yaw)
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
        self.sock.sendall("<t,{0}>".format(time))
        
    def send_time_and_position(self, time, x, y, z):
        '''Send both time and position at same time.'''
        self.sock.sendall("<tp,{0},{1},{2},{3}>".format(time, x, y, z))
        
    def send_time_position_and_orientation(self, time, x, y, z, angle1, angle2, angle3):
        '''Send time, position and orientation at same time.'''
        self.sock.sendall("<tpo,{0},{1},{2},{3},{4},{5},{6}>".format(time, x, y, z, angle1, angle2, angle3))
        
    def send_command_by_type(self, sensor_type, command):
        '''Send command to all sensors of specified type.'''
        self.sock.sendall("<ct,{0},{1}>".format(sensor_type, command))
        
    def send_command_by_name(self, sensor_name, command):
        '''Send command to all sensors with matching name (names aren't unique)'''
        self.sock.sendall("<cn,{0},{1}>".format(sensor_name, command))
        
    def send_command_by_id(self, sensor_id, command):
        '''Send command to sensor with matching ID. All ID's are unique.'''
        self.sock.sendall("<ci,{0},{1}>".format(sensor_id, command))

if __name__ == '__main__':
    
    rospy.init_node('sensor_control_client')
    
    default_server_host = socket.gethostname()
    default_server_port = 5000
    default_host = '{0} {1}'.format(default_server_host, default_server_port)

    # Determine if host is file path or a list.
    hosts = rospy.get_param('~sensor_hosts', default_host)
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
        
    rospy.Subscriber("gps_utc", OdometryUTC, odom_callback)
    
    rospy.spin()                               
