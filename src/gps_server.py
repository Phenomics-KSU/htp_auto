#!/usr/bin/env python

import sys
import os.path
import socket
import rospy
#import libnmea_navsat_driver.driver
import tf
import time
import threading
from Queue import Queue

from htp_auto.msg import OdometryUTC

def odom_callback(odom, server):
    '''Subscriber callback for odometery message. Pass along data to server.'''
    pose = odom.odom.pose.pose
    # Report UTM coordinates instead of local ENU so can geo-reference sensors to geoid.
    x = odom.easting #pose.position.x 
    y = odom.northing #pose.position.y
    z = odom.altitude #pose.position.z
    zone = odom.utm_zone
    utc_time = odom.time
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    roll = euler[0]
    pitch = 'nan' # euler[1] pitch isn't being measured so don't want to store it as 0 in database.
    yaw = euler[2]
    # Get system time when gps data was first read in.
    # Use half of merge time since that will estimate an inbetween of when position and orientation were measured.
    sys_time = odom.odom.header.stamp.to_sec() + (odom.merge_time_diff / 2.0)
    
    server.new_position(utc_time, sys_time, x, y, z, zone)
    server.new_orientation(utc_time, sys_time, roll, pitch, yaw)

class GPSServer(threading.Thread):
    '''
    UDP server that allows clients to connect and, essentially subscribe, to 
    new data that is posted to the server.
    '''

    def __init__(self, host, port):
        '''Constructor.'''
        super(GPSServer, self).__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)
        self.handlers = {}
        self.handlers_lock = threading.Lock()

    def run(self):
        '''
        Thread start method. Bind socket, then wait for new clients to connect.  
        If received a  new client then create a new handler for it (on a separate thread)
        if we don't already have one for it.
        '''
        # Set socket to be re-usable to avoid timeout after closing.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.address))
    
        while True:
            data, addr = self.sock.recvfrom(1024)

            with self.handlers_lock: 
                already_registered = addr in self.handlers
                if data == 'sync':
                    if already_registered:
                        self.handlers[addr].resync()
                    else:
                        self._create_new_handler(addr, sync=True)
                    # Tell client that its request has been received successfully.
                    self.sock.sendto('ack', addr)
                elif data =='add':
                    if not already_registered:
                        self._create_new_handler(addr, sync=False)
                    # Tell client that its request has been received successfully.
                    self.sock.sendto('ack', addr)
                    
    def new_time(self, utc_time, sys_time):
        '''
        Post new time to server.  This will send it out to all clients.
        Sys time is the system time when the UTC time was first read in.
        '''
        with self.handlers_lock:
            for handler in self.handlers.itervalues():
                handler.send_time(utc_time, sys_time)
                
    def new_position(self, utc_time, sys_time, x, y, z, zone=None):
        '''
        Post new time/position to server.  This will send it out to all clients.
        Sys time is the system time when the UTC time was first read in.
        Zone is for frames that are split into zones.  For example in UTM it could be 14S.
        '''
        with self.handlers_lock:
            for handler in self.handlers.itervalues():
                handler.send_position(utc_time, sys_time, x, y, z, zone)
                    
    def new_orientation(self, utc_time, sys_time, roll, pitch, yaw):
        '''
        Post new time/orientation to server. This will send it out to all clients.
        Sys time is the system time when the UTC time was first read in.
        Roll pitch in yaw are the relative rotations ZYX or static rotations XYZ.
        '''
        with self.handlers_lock:
            for handler in self.handlers.itervalues():
                handler.send_orientation(utc_time, sys_time, roll, pitch, yaw)
                    
    def _create_new_handler(self, addr, sync):
        '''Create client handler on background thread.'''
        handler = ClientHandler(addr, sync)
        handler.setDaemon(True)
        handler.start()
        self.handlers[addr] = handler
        return handler
        

class ClientHandler(threading.Thread):
    '''
    UDP handler that sends time, pose and commands to client. This will create a new
    socket to talk with client (since this is a different thread than the server),
    but as far as the client is concerned it can just recvfrom the server socket.
    '''
    
    def __init__(self, client_address, need_to_sync):
        super(ClientHandler, self).__init__()
        # SOCK_DGRAM is the socket type to use for UDP sockets
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = client_address
        
        # Set to True once host time is synced.
        self.synced = not need_to_sync
        
        # Last utc time used in sync command. Used to avoid syncing using duplicate timestamps.
        self.last_sync_time = 0 
        
        # Next id to use for sync command. Used to uniquely specify each sync message.
        self.next_sync_id = 0
        
        # Dict of {id, system_time} used for calculating elapsed time during sync process.
        self.sync_id_times = {}
        
        # Threadsafe queue for getting new data from server.
        self.queue = Queue()
        
    def run(self):
        '''Thread start method.  Send queued up data to client'''
        while True:
            # Block here until we have new data to handle.
            data = self.queue.get(block=True, timeout=None)
            try:
                # Update data for the elapsed time since it was read in.
                self._account_for_time_delay(data)
                
                if not self.synced:
                    self.synced = self._try_sync(data)
                
                if self.synced: 
                    data = ','.join([str(f) for f in data])
                    self.sock.sendto(data, self.address)
            except socket.error:
                rospy.logwarn('Socket error: client could not send data.')
    
    def send_time(self, utc_time, time_delay):
        '''Queue up time to be sent to client.'''
        self.queue.put(['t', utc_time, time_delay])
        
    def send_position(self, utc_time, time_delay, x, y, z, zone=None):
        '''Queue up time/position to be sent to client.'''
        self.queue.put(['p', utc_time, time_delay, x, y, z, zone])
        
    def send_orientation(self, utc_time, time_delay, roll, pitch, yaw):
        '''Queue up time/orientation to be sent to client.'''
        self.queue.put(['o', utc_time, time_delay, roll, pitch, yaw])
        
    def send_command_by_type(self, sensor_type, command):
        '''
        Queue up command to be sent to client.
        'sensor_type' is the type of sensors to send command to.  For example canon_mcu.
        'command' could be anything depending on sensor type.
        '''
        self.queue.put(['ct', sensor_type, command])
        
    def send_command_by_name(self, sensor_name, command):
        '''
        Queue up command to be sent to client.
        Will send to all sensors with matching name (names aren't unique)
        '''
        self.queue.put(['cn', sensor_name, command])
        
    def send_command_by_id(self, sensor_id, command):
        '''
        Queue up command to be sent to client.
        Will send to all sensors with matching ID. All ID's are unique for a given run.
        '''
        self.queue.put(['ci', sensor_id, command])
                    
    def resync(self):
        '''
        If not currently syncing time, then will force to resync to client. If already syncing
        then this won't do anything.  Useful for if a client is restarted with the same address.
        Thread-safe since setting bool is atomic.  
        '''
        self.synced = False
               
    def _account_for_time_delay(self, data):
        '''
        If data contains a time reference (should be at index 1), then switch out the 'sys_time' reference
        (should be at index 2) for the elapsed time (the time delay) since the data was originally read in.
        '''
        
        # Make sure message has a sys time reference to use
        if (len(data) <= 2) or (data[0] not in ['t', 'p', 'o']):
            return False 
                   
        # Calculate time that's elapsed since UTC time was read in.
        sys_time_ref = data[2]
        elapsed_time = rospy.get_time() - sys_time_ref
  
        # Replace sys time ref with elapsed times.
        data[2] = elapsed_time
                    
    def _try_sync(self, data):
        '''
        If 'data' is a message that can be used for time syncing to client then updates
        sync process.  If the time is already been used then it will be ignored.  Must
        call account_for_time_delay() before this.
        '''
        if (len(data) <= 1) or (data[0] not in ['t', 'p', 'o']):
            return False # can't use this message for a sync
        
        sync_time = data[1]
         
        if sync_time == self.last_sync_time:
            return False # already used this same time to sync 
        
        self.last_sync_time = sync_time
        
        # Add on any time delay.
        if len(data) >= 3:
            sync_time += data[2]
        
        return self._send_time_sync(sync_time)
                    
    def _send_time_sync(self, utc_time):
        '''
        More robust way of making sure that the UTC time the sensor control uses is accurate.
        To use this method effectively the client should be setup with the RelativePreciseTimeSource.
        This method uses the following steps to greatly reduce the latency effects between the client and server.
        
        This method should be called for each new UTC time until it reports success (returns true). 
        
        Handler: Records system time and id number
        Handler: Sends UTC time (with processing delay added in) and id number
        Client: Stores UTC time + id number + its system clock
        Client: Acknowledges packet by sending back same ID number.
        Handler: Calculates round trip time (RTT) and then sends back same ID and RTT value.
        Client: Cuts RTT value in half to estimate latency and adds it to the original UTC time it stored with the ID and system clock.
        Client: Checks if it has at least 10 messages.   If it does then adds checks if they're all consistent.  If they are then sends back 'true'. 
        
        This method returns true if client is successfully synced.  After that orientation and position messages can start being sent.
        '''
        sync_id = self.next_sync_id
        self.sock.settimeout(1.0)
        self.sock.sendto("sync1,{},{}".format(sync_id, utc_time), self.address)
        self.sync_id_times[sync_id] = rospy.get_time()

        data, _ = self.sock.recvfrom(1024)

        returned_sync_id = int(data)
        returned_sync_time = rospy.get_time()
        sent_sync_time = self.sync_id_times[returned_sync_id]
        elapsed_time = returned_sync_time - sent_sync_time
        
        self.sock.sendto("sync2,{},{}".format(sync_id, elapsed_time), self.address)
        
        data, _ = self.sock.recvfrom(1024)
        if data == 'true':
            self.synced = True
            
        self.sock.settimeout(None)
        self.next_sync_id += 1
        
        return self.synced
    
if __name__ == '__main__':
    
    rospy.init_node('gps_server')

    host = rospy.get_param('~sh', "0.0.0.0")
    server_port = rospy.get_param('~sp', 50005)

    if host == 'hostname':
        host = socket.gethostname()

    rospy.loginfo("Starting server at {}:{}".format(host, server_port))
    server = GPSServer(host, server_port)
    server.setDaemon(True)
    server.start()
    
    # Keep buffer size at 1 to avoid buffering old messages.
    rospy.Subscriber("gps_utc_throttled", OdometryUTC, odom_callback, queue_size = 1, callback_args=server)
    
    rospy.spin()                               
