import logging
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.wait_for_message import wait_for_message

import subprocess
import time
import base64
import json

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

import os
import signal

process_bringup = None
process_mapping = None


def read_from_sensor():

    battery_percent = None
    
    class BatteryRead(Node):
        def __init__(self):
            super().__init__('battery_read')

            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )

            self.subscription = self.create_subscription(
                BatteryState, '/mavros/battery', self.battery_callback, qos_profile)

        def battery_callback(self, msg):
            nonlocal battery_percent
            if msg.voltage < 13.6:
                battery_percent = 0
            elif msg.voltage == 16.8: 
                battery_percent = 100
            else: 
                battery_percent = ((msg.voltage - 13.6) / (16.8 - 13.6)) * 100
            self.get_logger().info(f'Battery: {battery_percent:.2f}%')
    
    def main():
        rclpy.init()
        battery_read = BatteryRead()
        rclpy.spin_once(battery_read, timeout_sec=1.0)
        battery_read.destroy_node()
        rclpy.shutdown()
    
    main()

    return battery_percent

# def read_from_sensor():
#     rclpy.init()
#     battery_read = BatteryRead()
#     print("Reading data from drone...")

#     battery_percent, gps_data, local_data = 0, [], []

#     # Spin for 2 seconds
#     # t_end = time.time() + 2.5
#     try:
#         # while time.time() < t_end:
#             # print(time.time())
#         # rclpy.spin_once(battery_read, timeout_sec=0.5)
        
#         # Battery
#         msg = wait_for_message(BatteryState, battery_read, "/mavros/battery", time_to_wait=0.5)
#         if msg[0] :
#             battery_percent = msg[1].voltage
#             if msg[1].voltage < 13.6:
#                 battery_percent = 0
#             elif msg[1].voltage == 16.8: 
#                 battery_percent = 100
#             else: 
#                 battery_percent = ((msg[1].voltage - 13.6) / (16.8 - 13.6)) * 100
#         # GPS
#         msg = wait_for_message(NavSatFix, battery_read, "/mavros/global_position/global", time_to_wait=1.5)
#         if msg[0] :
#             gps_data = [msg[1].latitude, msg[1].longitude, msg[1].altitude]
#         else :
#             print("No GPS data")
#         # Local data
#         msg = wait_for_message(Odometry, battery_read, "/mavros/global_position/local", time_to_wait=1.5)
#         if msg[0] :
#             local_data = [msg[1].pose.pose.position.x, msg[1].pose.pose.position.y, msg[1].pose.pose.position.z]
#         time.sleep(0.1)
#     finally:
#         battery_read.destroy_node()
#         rclpy.shutdown()
#     print("Done!")
#     # executor = MultiThreadedExecutor()
#     # executor.add_node(battery_read)
#     # executor.spin()  # Run in a multi-threaded mode

#     # battery_read.destroy_node()
#     # rclpy.shutdown()
    
#     # return battery_read.battery_percent, battery_read.gps_data, battery_read.local_data
#     return battery_percent, gps_data, local_data

def read_from_gps_sensor():
    
    altitude = None 
    longitude = None 
    latitude = None 

    class GpsRead(Node):
        def __init__(self):
            super().__init__('gps_read')

            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)          
            self.subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)

        def gps_callback(self, msg):
            nonlocal altitude
            nonlocal latitude
            nonlocal longitude
            altitude = msg.altitude
            latitude = msg.latitude
            longitude = msg.longitude
            self.get_logger().info(f"GPS: {latitude}, {longitude}, {altitude}")

    def main():
        rclpy.init()
        gps_read = GpsRead()
        rclpy.spin_once(gps_read, timeout_sec=1.0)
        gps_read.destroy_node()
        rclpy.shutdown()

    main()

    return altitude, latitude, longitude

def read_from_local_sensor():
    
    pos_x = None
    pos_y = None
    pos_z = None

    class LocalRead(Node):
        def __init__(self):
            super().__init__('localpose_read')
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
            self.subscription = self.create_subscription(Odometry, '/mavros/global_position/local', self.gps_callback, qos_profile)

        def gps_callback(self, msg):
            nonlocal pos_x
            nonlocal pos_y
            nonlocal pos_x
            pos_x = msg.pose.pose.position.x
            pos_y = msg.pose.pose.position.y
            pos_z = msg.pose.pose.position.z
            self.get_logger().info(f"Local Position: x = {pos_x}, y = {pos_y}, z = {pos_y}")

    def main():
        rclpy.init()
        local_read = LocalRead()
        rclpy.spin_once(local_read, timeout_sec=1.0)
        local_read.destroy_node()
        rclpy.shutdown()

    main()

    return pos_x, pos_y, pos_z

allAvailableResources_init = {
    'battery_percent': read_from_sensor(),
    'gps_data': read_from_gps_sensor(),
    'local_data': read_from_gps_sensor()
}

possibleLaunchfiles_drone_init = ['stopmapping_drone', 'startmapping_drone', 'bringup_zed', 'stopzed_drone', 'save3dmap_drone', 'savemap_drone', 'savebag_drone', 'stopbag_drone']
mapdataExportTF_init = [True, False]

def get_map_as_string(map_file_path):
    try:
        # Read the PGM file as binary
        with open(map_file_path, 'rb') as file:
           pgm_data = file.read()

        # Convert the PGM binary data to a string
        pgm_string = base64.b64encode(pgm_data).decode('utf-8')
        # # Open the file in binary mode and read it in chunks
        # with open(map_file_path, 'rb') as file:
        #     chunks = []
        #     while chunk := file.read(8192):  # Read in 8KB chunks
        #         chunks.append(base64.b64encode(chunk).decode('utf-8'))
        
        # # Join all the encoded chunks into a single string
        # png_string = ''.join(chunks)
        return pgm_string

    except FileNotFoundError:
        print("Error: Map file not found.")
        return None

def get_3dmap_as_string(map_file_path):
    try:
        # Read the bag file as binary
        with open(map_file_path, 'rb') as file:
            chunks = []
            while chunk := file.read(8192):  # Read in 8KB chunks
                chunks.append(base64.b64encode(chunk).decode('utf-8'))
        
        # Join all the encoded chunks into a single string
        png_string = ''.join(chunks)
        return png_string
    except FileNotFoundError:
        print("Error: Bagfile not found.")
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
    battery_info = read_from_sensor()

    batterypercent = battery_info if battery_info is not None else None
    print(f'Battery Percentage: {batterypercent}%')
    bringupaction = None
    stopzedaction = None
    mappingaction = None
    stopmappingaction = None
    mapping3dsaveaction = None
    saveaction = None
    savebagaction = None
    stopbagaction = None
    global process_bringup
    global process_mapping
    
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
        if process_bringup:
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
            print("zed camera stopped.")
        else:
            print("No zed camera process running.")
            stopzedaction = False
        process_bringup = None  # Reset the process variable

    if launchfileId == 'startmapping_drone':
        # If battery percentage is more than 50, allow to start the mapping launch file
        # print("Battery sufficient, start drone mapping!")
        process_mapping = subprocess.Popen(['ros2', 'launch', 'pc2_to_grid', 'grid_mapping_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5) 

        if process_mapping.poll() is None:
            print("Mapping started successfully.")
            mappingaction = True
        else:
            print("Failed to start mapping.")
            mappingaction = False

    if launchfileId == 'stopmapping_drone':
        if process_mapping:
            # Ensure that the process exists and is running
            if process_mapping.poll() is None:
                try:
                    # Gracefully terminate the process
                    process_mapping.send_signal(signal.SIGINT)
                    process_mapping.wait(timeout=30)
                    print("Process terminated gracefully.")
                except subprocess.TimeoutExpired:
                    # Forcefully kill the process if it didn't terminate
                    print("Process did not terminate in time. Killing it forcefully.")
                    process_mapping.kill()
                    process_mapping.wait()
                except Exception as e:
                    print(f"An error occurred: {e}")
                stopmappingaction = True
            print("Mapping stopped.")
        else:
            print("No mapping process running.")
            stopmappingaction = False
        process_mapping = None  # Reset the process variable
    
    if launchfileId == 'save3dmap_drone':
        process_3dmapping = subprocess.Popen(['ros2', 'launch', 'pc2_to_grid', 'pc2_map_png_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5) 

        if process_3dmapping.poll() is None:
            print("3D Mapping saved successfully.")
            mapping3dsaveaction = True
        else:
            print("Failed to save the 3D map.")
            mapping3dsaveaction = False

    if launchfileId == 'savemap_drone': #and mappingaction == True:
        print("Mapping finished, save the map!")
        process_savemapping = subprocess.Popen(['ros2', 'launch', 'pc2_to_grid', 'map_saver_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5) 
       
        print("Map saved successfully.")
        saveaction = True
        
    if launchfileId == 'savebag_drone':
        print("Starting recording rosbag!")
        global process_bagrecording
        process_bagrecording = subprocess.Popen(['exec ros2 bag record -s mcap -o my_bag -d 20 -b 50000 -a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
        time.sleep(5) 
       
        print("Bag recording started.")
        savebagaction = True
    
    if launchfileId == 'stopbag_drone':
        print("Stopping recording rosbag!")
        if process_bagrecording.poll() is None:
            process_bagrecording.terminate()
            process_bagrecording.wait()
            time.sleep(5)
        #print(process_bagrecording)
        #process_bagrecording.terminate()#kill()
        #os.killpg(process_bagrecording, signal.SIGTERM)
        print("Bag recording stopped.")
        stopbagaction = True

    # Read the current level of allAvailableResources_drone
    resources = await exposed_thing.read_property('allAvailableResources_drone')

    # Calculate the new level of resources
    newResources = resources.copy()
    
    battery_percent = read_from_sensor()
    
    newResources['battery_percent'] = battery_percent
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
    elif launchfileId == 'stopzed_drone':
        return {'result': stopzedaction, 'message': f'Your {launchfileId} is in progress!'}
    elif launchfileId == 'save3dmap_drone':
        return {'result': mapping3dsaveaction, 'message': f'Your {launchfileId} is in progress!'}
    
async def map3dExport_drone_handler(params):
    params = params['input'] if params['input'] else {}
    map_file_path = '/home/orin/my_3dmap.png'
    map_string = get_3dmap_as_string(map_file_path)
    return map_string

async def mapExport_drone_handler(params):
    params = params['input'] if params['input'] else {}
    map_file_path = '/home/orin/my_map.pgm'
    map_string = get_map_as_string(map_file_path)
    return map_string

async def bagExport_drone_handler(params):
    params = params['input'] if params['input'] else {}
    bag_file_path = '/home/orin/my_bag/my_bag_0.mcap'
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
    
    allAvailableResources_current = {
    'battery_percent': read_from_sensor(),
    'gps_data': read_from_gps_sensor(),
    'local_data': read_from_local_sensor()
    }

    return allAvailableResources_current

async def currentValues_drone_handler(params):
    
    return {
        'result': True,
        'message': {
            'battery_percent': read_from_sensor(),
            'gps_data': read_from_gps_sensor(),
            'local_data': read_from_local_sensor()
        }
    }

