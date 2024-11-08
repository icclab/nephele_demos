#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio

import requests
from flask import Flask, render_template, request, send_file

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT
from wotpy.protocols.http.client import HTTPClient


from PIL import Image
from io import BytesIO
import base64

import logging

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)


app = Flask(__name__,
            static_folder='/app')


@app.route('/')
def index():
    return render_template('index.html')

http_client = HTTPClient()
security_scheme_dict = {
    "scheme": "bearer"
}
credentials_dict = {
    "token": "token"
}
http_client.set_security(security_scheme_dict, credentials_dict)
wot = WoT(servient=Servient(clients=[http_client]))


##
##  SUMMIT RELATED FUNCTIONS
##
    
@app.route('/trigger_execution_summit', methods=['POST'])
def trigger_execution_summit():
    launchfile_id_summit = request.form['launchfile_id_summit']
    result_summit = asyncio.run(trigger_summit(launchfile_id_summit))
    print("trigger_summit", result_summit)
    return render_template('index.html', execution_status=result_summit)

async def trigger_summit(launchfile_id_summit):
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    result_summit = await consumed_thing_summit.invoke_action("triggerBringup_summit", {'launchfileId': launchfile_id_summit }) # desired_launch_file_id=[bringup, startmapping, saveMap]
    return result_summit



    
@app.route('/current_values_summit', methods=['GET'])
def current_values_summit():
    result_summit = asyncio.run(current_summit())
    print("current_summit", result_summit)
    return render_template('index.html', current_status_summit=result_summit)

async def current_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    result_summit = await consumed_thing_summit.invoke_action("currentValues_summit") 
    return result_summit

@app.route("/read_data_summit", methods=['GET'])
def read_data_summit():
    result_summit = asyncio.run(read_summit())
    print("read_summit", result_summit)
    return render_template('index.html', data=result_summit)

async def read_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    result_summit = await consumed_thing_summit.properties["allAvailableResources_summit"].read()
    #result = await consumed_thing.invoke_action("currentValues")
    print(result_summit)
    return result_summit



@app.route('/map_export_summit', methods=['GET'])
def map_export_summit():
    result_summit = asyncio.run(export_map_summit())
    return render_template('index.html', image_url=result_summit)

async def export_map_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    result_summit = await consumed_thing_summit.invoke_action("mapExport_summit")
    if result_summit is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_summit)
    # Open PGM data as an image
        image_path_summit = '/app/image_summit.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_summit, format="PNG")
        return image_path_summit
    



@app.route("/read_data_db_summit", methods=['GET'])
def read_data_db_summit():
    result_summit = asyncio.run(read_db_summit())
    print("read_db", result_summit)
    return render_template('index.html', data_db=result_summit)

async def read_db_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    
    filename = request.args.get('filename')

    result_summit = await consumed_thing_summit.invoke_action("filenamesReadDB_summit")
    return result_summit


    
    
@app.route("/store_map_db_summit", methods=['GET'])
def store_map_db_summit():
    result_summit = asyncio.run(storemapdb_summit())
    return render_template('index.html', store_map_db_summit=result_summit)

async def storemapdb_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    filename_tosave_summit = request.args.get('filename_tosave')
    result_summit = await consumed_thing_summit.invoke_action("mapStoreDB_summit", {'filename_tosave': filename_tosave_summit }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_summit)
    return result_summit


@app.route("/store_bag_vo_summit", methods=['GET'])
def store_bag_vo_summit():
    result_summit = asyncio.run(storebagvo_summit())
    return render_template('index.html', store_bag_vo=result_summit)

async def storebagvo_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    bagname_tosave_summit = request.args.get('bagname_tosave')
    result_summit = await consumed_thing_summit.invoke_action("bagStoreVO_summit", {'filename_tosave': bagname_tosave_summit }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_summit)
    return result_summit


@app.route("/read_map_from_db_summit", methods=['GET'])
def read_map_from_db_summit():
    result_summit = asyncio.run(read_map_db_summit())
    return render_template('index.html', map_from_db=result_summit)


async def read_map_db_summit():
    consumed_thing_summit = await wot.consume_from_url("http://cvo:9090/cvo")
    
    # Get the filename from the query parameters
    filename_map_summit = request.args.get('filename_map')

    #result = await consumed_thing.read_property("someStringProperty")
    result_summit = await consumed_thing_summit.invoke_action("mapReadDB_summit", {'filename_map': filename_map_summit })
    if result_summit is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_summit)
    # Open PGM data as an image
        image_path_db_summit = '/app/image_map_db_summit.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_db_summit, format="PNG")
        return image_path_db_summit


##
##  DRONE RELATED FUNCTIONS
##

   
@app.route('/trigger_execution_drone', methods=['POST'])
def trigger_execution_drone():
    launchfile_id_drone = request.form['launchfile_id_drone']
    result_drone = asyncio.run(trigger_drone(launchfile_id_drone))
    print("trigger_drone", result_drone)
    return render_template('index.html', execution_status=result_drone)

async def trigger_drone(launchfile_id_drone):
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    result_drone = await consumed_thing_drone.invoke_action("triggerBringup_drone", {'launchfileId': launchfile_id_drone }) # desired_launch_file_id=[bringup, startmapping, saveMap]
    return result_drone

    
@app.route('/current_values_drone', methods=['GET'])
def current_values_drone():
    result_drone = asyncio.run(current_drone())
    print("current_drone", result_drone)
    return render_template('index.html', current_status_drone=result_drone)

async def current_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    result_drone = await consumed_thing_drone.invoke_action("currentValues_drone") 
    return result_drone

@app.route("/read_data_drone", methods=['GET'])
def read_data_drone():
    result_drone = asyncio.run(read_drone())
    print("read_drone", result_drone)
    return render_template('index.html', data=result_drone)

async def read_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    result_drone = await consumed_thing_drone.properties["allAvailableResources_drone"].read()
    #result = await consumed_thing.invoke_action("currentValues")
    print(result_drone)
    return result_drone



@app.route('/map_export_drone', methods=['GET'])
def map_export_drone():
    result_drone = asyncio.run(export_map_drone())
    return render_template('index.html', image_url=result_drone)

async def export_map_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    result_drone = await consumed_thing_drone.invoke_action("mapExport_drone")
    if result_drone is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_drone)
    # Open PGM data as an image
        image_path_drone = '/app/image_drone.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_drone, format="PNG")
        return image_path_drone
    

@app.route("/read_data_db_drone", methods=['GET'])
def read_data_db_drone():
    result_drone = asyncio.run(read_db_drone())
    print("read_db", result_drone)
    return render_template('index.html', data_db=result_drone)

async def read_db_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo") 
    filename = request.args.get('filename')
    result_drone = await consumed_thing_drone.invoke_action("filenamesReadDB_drone")
    return result_drone
    
@app.route("/store_map_db_drone", methods=['GET'])
def store_map_db_drone():
    result_drone = asyncio.run(storemapdb_drone())
    return render_template('index.html', store_map_db_drone=result_drone)

async def storemapdb_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    filename_tosave_drone = request.args.get('filename_tosave')
    result_drone = await consumed_thing_drone.invoke_action("mapStoreDB_drone", {'filename_tosave': filename_tosave_drone }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_drone)
    return result_drone


@app.route("/store_bag_vo_drone", methods=['GET'])
def store_bag_vo_drone():
    result_drone = asyncio.run(storebagvo_drone())
    return render_template('index.html', store_bag_vo=result_drone)

async def storebagvo_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    bagname_tosave_drone = request.args.get('bagname_tosave')
    result_drone = await consumed_thing_drone.invoke_action("bagStoreVO_drone", {'filename_tosave': bagname_tosave_drone }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_drone)
    return result_drone


@app.route("/read_map_from_db_drone", methods=['GET'])
def read_map_from_db_drone():
    result_drone = asyncio.run(read_map_db_drone())
    return render_template('index.html', map_from_db=result_drone)


async def read_map_db_drone():
    consumed_thing_drone = await wot.consume_from_url("http://cvo:9090/cvo")
    
    # Get the filename from the query parameters
    filename_map_drone = request.args.get('filename_map')

    #result = await consumed_thing.read_property("someStringProperty")
    result_drone = await consumed_thing_drone.invoke_action("mapReadDB_drone", {'filename_map': filename_map_drone })
    if result_drone is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_drone)
    # Open PGM data as an image
        image_path_db_drone = '/app/image_map_db_drone.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_db_drone, format="PNG")
        return image_path_db_drone

if __name__ == "__main__":
    app.run(debug=True)

