import logging
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from px4_msgs.msg import BatteryStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import subprocess
import time
import base64

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

import os
import signal

def read_from_sensor():
    # if sensorType != 'kobuki: Battery':
    #     print(f"Sensor type '{sensorType}' is not supported.")
    #     return None
    
    battery_percent = None 
    # battery_charging = None

    class BatteryRead(Node):
        def __init__(self):
            super().__init__('battery_read')
            
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

            self.subscription = self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile)
            # self.subscription = self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, 10)

        def battery_callback(self, msg):
            nonlocal battery_percent
            # nonlocal battery_charging
            if msg.voltage_v < 13.6:
                battery_percent = 0
            elif msg.voltage_v == 16.8: 
                battery_percent = 100
            else: 
                battery_percent = ((msg.voltage_v - 13.6) / (16.8 - 13.6)) * 100

    def main():
        rclpy.init()
        battery_read = BatteryRead()
        rclpy.spin_once(battery_read, timeout_sec=1.0)
        battery_read.destroy_node()
        rclpy.shutdown()

    main()

    return battery_percent
    
allAvailableResources_init = {
    'battery_percent': read_from_sensor()
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

async def triggerBringup_zed_handler(params):
    params = params['input'] if params['input'] else {}

    # Default values
    launchfileId = 'startmapping_drone'

    # Check if params are provided
    launchfileId = params.get('launchfileId', launchfileId)

    # Check if there is resources
    #battery_info = read_from_sensor('HDD Usage (SXLS0_180227AA)')
    battery_info = read_from_sensor()
    #batterypercent = battery_info[0] if battery_info is not None else None
    batterypercent = battery_info if battery_info is not None else None
    #batterycharging = battery_info[1] if battery_info is not None else None
    print(f'Battery Percentage: {batterypercent}%')
    #print(f'Battery is charging: {batterycharging}')
    bringupaction = None
    mappingaction = None
    saveaction = None
    savebagaction = None
    stopbagaction = None
    #process_bagrecording = None
    

    #if launchfileId == 'bringup' and batterypercent is None :
    if launchfileId == 'bringup_zed':
        # If battery percentage is None, start the drone launch file
        print("Start Zed camera!")
        process_bringup = subprocess.Popen(['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py', 'camera_model:=zed2i'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Allow some time for the launch file to start
        time.sleep(20)  

        # Check if the process is still running
        if process_bringup.poll() is None:
            print("Launch file started successfully.")
            bringupaction = True
        else:
            print("Failed to start the launch file.")
            bringupaction = False

   # if launchfileId == 'startmapping' and batterypercent >= 30:
    if launchfileId == 'startmapping_drone':
        # If battery percentage is more than 50, allow to start the mapping launch file
        # print("Battery sufficient, start drone mapping!")
        #process_mapping = subprocess.Popen(['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
       # process_mapping = subprocess.Popen(['ros2', 'launch', 'drone_xl_navigation', 'nav2_bringup_launch.py', 'use_sim_time:=false', 'slam:=True'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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

            
        


       # if process_savemapping.poll() is None:
       #     print("Map saved successfully.")
       #     saveaction = True
       # else:
       #     print("Failed to save map.")
       #     saveaction = False
    

    # Read the current level of allAvailableResources_drone
    resources = await exposed_thing.read_property('allAvailableResources_drone')

    # Calculate the new level of resources
    newResources = resources.copy()
    newResources['battery_percent'] = read_from_sensor()
   # newResources['battery_charging'] = read_from_sensor('HDD Usage (SXLS0_180227AA)')[1]
    
    # Check if the amount of available resources is sufficient to launch
    if newResources['battery_percent'] <= 30:
        # Emit outOfResource event
        exposed_thing.emit_event('outOfResource_drone', 'Low level of Battery Percentage')
        return {'result': False, 'message': 'battery is not sufficient'}
    
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
    
async def allAvailableResources_drone_read_handler():
    allAvailableResources_current = {
    'battery_percent': read_from_sensor()
 #   'battery_percent': read_from_sensor('HDD Usage (SXLS0_180227AA)')[0],
 #   'battery_charging': read_from_sensor('HDD Usage (SXLS0_180227AA)')[1],
    }

    return allAvailableResources_current

async def currentValues_drone_handler(params):
    return {
        'result': True,
        'message': {
    'battery_percent': read_from_sensor()
 #   'battery_percent': read_from_sensor('HDD Usage (SXLS0_180227AA)')[0],
 #   'battery_charging': read_from_sensor('HDD Usage (SXLS0_180227AA)')[1],
        }
    }


