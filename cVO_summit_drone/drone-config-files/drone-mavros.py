import logging
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import subprocess
import time
import base64
import json

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

import os
import signal

def read_from_sensor():  
    battery_percent = None 
    gps_data = []
    local_data = []

    class BatteryRead(Node):
        def __init__(self):
            super().__init__('battery_read')
            
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

            self.subscription = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile)
            self.gps_subscription = self.create_subscription(MarkerArray, '/gps_marker', self.gps_callback, 10)
            self.localPostion_subscription = self.create_subscription(MarkerArray, '/local_marker', self.local_callback, 10)

        def battery_callback(self, msg):
            nonlocal battery_percent
            if msg.voltage < 13.6:
                battery_percent = 0
            elif msg.voltage == 16.8: 
                battery_percent = 100
            else: 
                battery_percent = ((msg.voltage - 13.6) / (16.8 - 13.6)) * 100

        def gps_callback(self, msg):
            nonlocal gps_data
            for marker in msg.markers:
                point = Point(
                    x=marker.pose.position.x,
                    y=marker.pose.position.y,
                    z=marker.pose.position.z
                )
                gps_data.append(point)

        def local_callback(self, msg):
            nonlocal local_data
            for marker in msg.markers:
                point = Point(
                    x=marker.pose.position.x,
                    y=marker.pose.position.y,
                    z=marker.pose.position.z
                )
                local_data.append(point)

    def main():
        rclpy.init()
        battery_read = BatteryRead()
        rclpy.spin_once(battery_read, timeout_sec=1.0)
        battery_read.destroy_node()
        rclpy.shutdown()

    main()

    return battery_percent, gps_data, local_data
    
battery_percent, gps_data, local_data = read_from_sensor()

allAvailableResources_init = {
    'battery_percent': battery_percent,
    'gps_data': gps_data,
    'local_data': local_data
}

possibleLaunchfiles_drone_init = ['startmapping_drone', 'bringup_zed', 'savemap_drone', 'savebag_drone', 'stopbag_drone']
mapdataExportTF_init = [True, False]

def get_map_as_string(map_file_path):
    try:
        # Read the PGM file as binary
        with open(map_file_path, 'rb') as file:
            pgm_data = file.read()

        # Convert the PGM binary data to a string
        pgm_string = base64.b64encode(pgm_data).decode('utf-8')

        return pgm_string

    except FileNotFoundError:
        print("Error: Map file not found.")
        return None
    
def get_rosbag_as_string(bag_file_path):
    try:
        # Read the bag file as binary
        with open(bag_file_path, 'rb') as file:
            bag_data = file.read()

        # Convert the MCAP binary data to a string ??? How???
        bag_string = base64.b64encode(bag_data).decode('utf-8')

        return bag_string

    except FileNotFoundError:
        print("Error: Bagfile not found.")
        return None

async def triggerBringup_drone_handler(params):
    params = params['input'] if params['input'] else {}

    # Default values
    launchfileId = 'startmapping_drone'

    # Check if params are provided
    launchfileId = params.get('launchfileId', launchfileId)

    # Check if there is resources
    battery_info, gps_data, local_data = read_from_sensor()
    batterypercent = battery_info if battery_info is not None else None
    print(f'Battery Percentage: {batterypercent}%')
    bringupaction = None
    stopzedaction = None
    mappingaction = None
    saveaction = None
    savebagaction = None
    stopbagaction = None
    
    if launchfileId == 'bringup_zed':
        print("Start Zed camera!")
        process_bringup = subprocess.Popen(['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Allow some time for the launch file to start
        time.sleep(20)  

        # Check if the process is still running
        if process_bringup.poll() is None:
            print("Launch file started successfully.")
            bringupaction = True
        else:
            print("Failed to start the launch file.")
            bringupaction = False

    if launchfileId == 'stopzed_drone':
        if process_startarmcamera:
            # Ensure that the process exists and is running
            if process_bringup.poll() is None:
                try:
                    # Gracefully terminate the process
                    process_bringup.send_signal(signal.SIGINT)
                    process_bringup.wait(timeout=30)
                    print("Process terminated gracefully.")
                except subprocess.TimeoutExpired:
                    # Forcefully kill the process if it didn't terminate
                    print("Process did not terminate in time. Killing it forcefully.")
                    process_bringup.kill()
                    process_bringup.wait()
                except Exception as e:
                    print(f"An error occurred: {e}")
                stopzedaction = True
            print("Arm camera stopped.")
        else:
            print("No arm camera process running.")
            stopzedaction = False
        process_bringup = None  # Reset the process variable

    if launchfileId == 'startmapping_drone':
        # If battery percentage is more than 50, allow to start the mapping launch file
        # print("Battery sufficient, start drone mapping!")
        process_mapping = subprocess.Popen(['ros2', 'launch', 'pc2_to_grid', 'pc2_transformed_to_grid_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(10) 

        if process_mapping.poll() is None:
            print("Mapping started successfully.")
            mappingaction = True
        else:
            print("Failed to start mapping.")
            mappingaction = False

    if launchfileId == 'savemap_drone': #and mappingaction == True:
        print("Mapping finished, save the map!")
        process_savemapping = subprocess.Popen(['ros2', 'launch', 'pc2_to_grid', 'map_saver_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(20) 
       
        print("Map saved successfully.")
        saveaction = True
        
    if launchfileId == 'savebag_drone':
        print("Starting recording rosbag!")
        global process_bagrecording
        process_bagrecording = subprocess.Popen(['exec ros2 bag record -s mcap -o my_bag -d 20 -b 50000 -a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
        time.sleep(20) 
       
        print("Bag recording started.")
        savebagaction = True
    
    if launchfileId == 'stopbag_drone':
        print("Stopping recording rosbag!")
        if process_bagrecording.poll() is None:
            process_bagrecording.terminate()
            process_bagrecording.wait()
            time.sleep(20)
        #print(process_bagrecording)
        #process_bagrecording.terminate()#kill()
        #os.killpg(process_bagrecording, signal.SIGTERM)
        print("Bag recording stopped.")
        stopbagaction = True

    # Read the current level of allAvailableResources_drone
    resources = await exposed_thing.read_property('allAvailableResources_drone')

    # Calculate the new level of resources
    newResources = resources.copy()
    battery_info, gps_data, local_data = read_from_sensor()
    newResources['battery_percent'] = battery_info
    # newResources['gps_data'] = gps_data
    # newResources['local_data'] = local_data
        
    # Check if the amount of available resources is sufficient to launch
    # if newResources['battery_percent'] <= 30:
    #     # Emit outOfResource event
    #     exposed_thing.emit_event('outOfResource_drone', 'Low level of Battery Percentage')
    #     return {'result': False, 'message': 'battery is not sufficient'}
    
    # Now store the new level of allAvailableResources_drone 
    await exposed_thing.properties['allAvailableResources_drone'].write(newResources)

    # Finally deliver the launchfile
    if launchfileId == 'bringup_zed':
        print('bringup finishded')
        return {'result': bringupaction, 'message': f'Your {launchfileId} is in progress!'}
    elif launchfileId == 'startmapping_drone':
        return {'result': mappingaction, 'message': f'Your {launchfileId} is in progress!'}
    elif launchfileId == 'savemap_drone':
        return {'result': saveaction, 'message': f'Your {launchfileId} is in progress!'}
    elif launchfileId == 'savebag_drone':
        return {'result': savebagaction, 'message': f'Your {launchfileId} is in progress!'}
    elif launchfileId == 'stopbag_drone':
         return {'result': stopbagaction, 'message': f'Your {launchfileId} is in progress!'}
    
async def mapExport_drone_handler(params):
    params = params['input'] if params['input'] else {}
    map_file_path = '/home/ros/my_map.pgm'
    map_string = get_map_as_string(map_file_path)
    return map_string

async def bagExport_drone_handler(params):
    params = params['input'] if params['input'] else {}
    bag_file_path = '/home/ros/my_bag/my_bag_0.mcap'
    bag_string = get_rosbag_as_string(bag_file_path)
    return bag_string
    
async def gpsExport_drone_handler(params):
    params = params.get('input', {}) if params else {}
    battery_percent, gps_data, local_data = read_from_sensor()
    gps_export_list = [{"lat": p.x, "lon": p.y, "alt": p.z} for p in gps_data]
    gps_export_string = json.dumps(gps_export_list)
    return gps_export_string

async def localExport_drone_handler(params):
    params = params.get('input', {}) if params else {}
    battery_percent, gps_data, local_data = read_from_sensor()
    local_export_list = [{"x": p.x, "y": p.y, "z": p.z} for p in local_data]
    local_export_string = json.dumps(local_export_list)
    return local_export_string

async def allAvailableResources_drone_read_handler():
    battery_percent, gps_data, local_data = read_from_sensor()

    allAvailableResources_current = {
    'battery_percent': battery_percent,
    'gps_data': gps_data,
    'local_data': local_data
    }

    return allAvailableResources_current

async def currentValues_drone_handler(params):
    battery_percent, gps_data, local_data = read_from_sensor()

    return {
        'result': True,
        'message': {
            'battery_percent': battery_percent,
            'gps_data': gps_data,
            'local_data': local_data
        }
    }

