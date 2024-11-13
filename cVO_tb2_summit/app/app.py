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

   
@app.route('/trigger_execution_tb2', methods=['POST'])
def trigger_execution_tb2():
    launchfile_id_tb2 = request.form['launchfile_id_tb2']
    result_tb2 = asyncio.run(trigger_tb2(launchfile_id_tb2))
    print("trigger_tb2", result_tb2)
    return render_template('index.html', execution_status=result_tb2)

async def trigger_tb2(launchfile_id_tb2):
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    result_tb2 = await consumed_thing_tb2.invoke_action("triggerBringup_tb2", {'launchfileId': launchfile_id_tb2 }) # desired_launch_file_id=[bringup, startmapping, saveMap]
    return result_tb2

    
@app.route('/current_values_tb2', methods=['GET'])
def current_values_tb2():
    result_tb2 = asyncio.run(current_tb2())
    print("current_tb2", result_tb2)
    return render_template('index.html', current_status_tb2=result_tb2)

async def current_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    result_tb2 = await consumed_thing_tb2.invoke_action("currentValues_tb2") 
    return result_tb2

@app.route("/read_data_tb2", methods=['GET'])
def read_data_tb2():
    result_tb2 = asyncio.run(read_tb2())
    print("read_tb2", result_tb2)
    return render_template('index.html', data=result_tb2)

async def read_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    result_tb2 = await consumed_thing_tb2.properties["allAvailableResources_tb2"].read()
    #result = await consumed_thing.invoke_action("currentValues")
    print(result_tb2)
    return result_tb2



@app.route('/map_export_tb2', methods=['GET'])
def map_export_tb2():
    result_tb2 = asyncio.run(export_map_tb2())
    return render_template('index.html', image_url=result_tb2)

async def export_map_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    result_tb2 = await consumed_thing_tb2.invoke_action("mapExport_tb2")
    if result_tb2 is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_tb2)
    # Open PGM data as an image
        image_path_tb2 = '/app/image_tb2.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_tb2, format="PNG")
        return image_path_tb2
    

@app.route("/read_data_db_tb2", methods=['GET'])
def read_data_db_tb2():
    result_tb2 = asyncio.run(read_db_tb2())
    print("read_db", result_tb2)
    return render_template('index.html', data_db=result_tb2)

async def read_db_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo") 
    filename = request.args.get('filename')
    result_tb2 = await consumed_thing_tb2.invoke_action("filenamesReadDB_tb2")
    return result_tb2
    
@app.route("/store_map_db_tb2", methods=['GET'])
def store_map_db_tb2():
    result_tb2 = asyncio.run(storemapdb_tb2())
    return render_template('index.html', store_map_db_tb2=result_tb2)

async def storemapdb_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    filename_tosave_tb2 = request.args.get('filename_tosave')
    result_tb2 = await consumed_thing_tb2.invoke_action("mapStoreDB_tb2", {'filename_tosave': filename_tosave_tb2 }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_tb2)
    return result_tb2


@app.route("/store_bag_vo_tb2", methods=['GET'])
def store_bag_vo_tb2():
    result_tb2 = asyncio.run(storebagvo_tb2())
    return render_template('index.html', store_bag_vo=result_tb2)

async def storebagvo_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    # Get the filename from the query parameters
    bagname_tosave_tb2 = request.args.get('bagname_tosave')
    result_tb2 = await consumed_thing_tb2.invoke_action("bagStoreVO_tb2", {'filename_tosave': bagname_tosave_tb2 }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result_tb2)
    return result_tb2


@app.route("/read_map_from_db_tb2", methods=['GET'])
def read_map_from_db_tb2():
    result_tb2 = asyncio.run(read_map_db_tb2())
    return render_template('index.html', map_from_db=result_tb2)


async def read_map_db_tb2():
    consumed_thing_tb2 = await wot.consume_from_url("http://cvo:9090/cvo")
    
    # Get the filename from the query parameters
    filename_map_tb2 = request.args.get('filename_map')

    #result = await consumed_thing.read_property("someStringProperty")
    result_tb2 = await consumed_thing_tb2.invoke_action("mapReadDB_tb2", {'filename_map': filename_map_tb2 })
    if result_tb2 is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result_tb2)
    # Open PGM data as an image
        image_path_db_tb2 = '/app/image_map_db_tb2.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_db_tb2, format="PNG")
        return image_path_db_tb2

if __name__ == "__main__":
    app.run(debug=True)

