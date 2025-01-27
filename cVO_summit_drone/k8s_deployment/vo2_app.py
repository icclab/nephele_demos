#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import asyncio
#import mcap
from wotpy.functions.functions import vo_status, device_status

LOGGER = logging.getLogger()

from io import BytesIO
import base64
LOGGER.setLevel(logging.INFO)

TABLE_NAME = "string_data_table"



import zenoh
import json
import time

config = zenoh.Config()
config.insert_json5("mode", json.dumps("client"))
router_url = "tcp/160.85.253.140:30447"
config.insert_json5("connect/endpoints", json.dumps([router_url]))
print("Opening Zenoh session...")
zenoh_session = zenoh.open(config)
zenoh.init_log_from_env_or("error")

async def myRosbagAction_summit_handler(params):
    params = params['input'] if params['input'] else {}
    
    data = params['data']
    key = "process_trigger"
    print(f"Declaring Publisher on '{key}'...")
    pub = zenoh_session.declare_publisher(key)
    payload = f"{data}"
    LOGGER.info('Published topic is {}'.format(key))
    LOGGER.info('Published message is {}'.format(payload))
    pub.put(payload.encode('utf-8'))
    time.sleep(1)
    return {'message': f'{payload} rosbag storing trigger has been processed on the VO!'}


async def filenamesReadDB_summit_handler(params):
    params = params['input'] if params['input'] else {}
    # Default values
    servient = exposed_thing.servient
    sqlite_db = servient.sqlite_db
    result=sqlite_db.execute_query("SELECT filename FROM string_data_table")
    return result

async def mapReadDB_summit_handler(params):
    params = params['input'] if params['input'] else {}
    # Default values
    filename_map_summit = 'test'
    filename_map_summit = params.get('filename_map_summit', filename_map_summit)
    LOGGER.info('Result after params is {}'.format(filename_map_summit))
    servient = exposed_thing.servient
    sqlite_db = servient.sqlite_db
    result=sqlite_db.execute_query("SELECT content FROM string_data_table WHERE filename='%s'" % filename_map_summit)
   # result= sqlite_db.execute_query("SELECT filename FROM string_data_table") 
    parsed_result=result[0][0]
    return parsed_result




async def mapStoreDB_summit_handler(params):
    params = params['input'] if params['input'] else {}
     # Default values
    filename_tosave_summit = 'map1'

    # Check if params are provided
    filename_tosave_summit = params.get('filename_tosave_summit', filename_tosave_summit)
    LOGGER.info('Consumed Thing: {}'.format(consumed_vos["summit"]))
    mapstring = await consumed_vos["summit"].invoke_action("mapExport_summit")
            
    servient = exposed_thing.servient
    sqlite_db = servient.sqlite_db
    content = mapstring

    columns = {
        "filename": "TEXT",
        "content": "TEXT"
    }
    sqlite_db.create_table_if_not_exists(TABLE_NAME, columns)
    result=sqlite_db.insert_data(TABLE_NAME, (filename_tosave_summit, content))
    
    return {'message': f'Your map storing on db is in progress!'}


async def bagStoreVO_summit_handler(params):
    params = params['input'] if params['input'] else {}
     # Default values
    bagname_tosave_summit = 'rosbag.mcap'

    # Check if params are provided
    bagname_tosave_summit = params.get('bagname_tosave_summit', bagname_tosave_summit)
    LOGGER.info('Consumed Thing: {}'.format(consumed_vos["summit"]))
    LOGGER.info('VO1 funciton')
    bagstring = await consumed_vos["summit"].invoke_action("bagExport_summit")
    LOGGER.info('VO1 funvton 2')
    LOGGER.info('Result after params is {}'.format(bagstring))

    
    if bagstring is None:
            return None
    else:
        rosbag_raw_data= base64.b64decode(bagstring)
        bag_path = '/pod-data/rosbag.mcap'
        with open(bag_path, 'wb') as file:
            file.write(rosbag_raw_data)
    
    
    return {'message': f'Your bag storing on VO is in progress!'}



async def read_property_from_summit():
    # Initialize the property values
    allAvailableResources_summit = await consumed_vos["summit"].properties['allAvailableResources_summit'].read()
    possibleLaunchfiles_summit = await consumed_vos["summit"].properties['possibleLaunchfiles_summit'].read()
    
    # Initialize the property values
    await exposed_thing.properties['allAvailableResources_summit'].write(allAvailableResources_summit)
    await exposed_thing.properties['possibleLaunchfiles_summit'].write(possibleLaunchfiles_summit)

