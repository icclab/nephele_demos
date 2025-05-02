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
   
    
@app.route('/trigger_execution', methods=['POST'])
def trigger_execution():
    launchfile_id = request.form['launchfile_id']
    result = asyncio.run(trigger(launchfile_id))
    print("trigger", result)
    return render_template('index.html', execution_status=result)

async def trigger(launchfile_id):
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    result = await consumed_thing.invoke_action("triggerBringup", {'launchfileId': launchfile_id }) # desired_launch_file_id=[bringup, startmapping, saveMap]
    return result



    
@app.route('/current_values', methods=['GET'])
def current_values():
    result = asyncio.run(current())
    print("current", result)
    return render_template('index.html', current_status=result)

async def current():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    result = await consumed_thing.invoke_action("currentValues") 
    return result

@app.route("/read_data", methods=['GET'])
def read_data():
    result = asyncio.run(read())
    print("read", result)
    return render_template('index.html', data=result)

async def read():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    result = await consumed_thing.properties["allAvailableResources"].read()
    #result = await consumed_thing.invoke_action("currentValues")
    print(result)
    return result



@app.route('/map_export', methods=['GET'])
def map_export():
    result = asyncio.run(export_map())
    return render_template('index.html', image_url=result)

async def export_map():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    result = await consumed_thing.invoke_action("mapExport")
    if result is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result)
    # Open PGM data as an image
        image_path = '/app/image.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path, format="PNG")
        return image_path
    



@app.route("/read_data_db", methods=['GET'])
def read_data_db():
    result = asyncio.run(read_db())
    print("read_db", result)
    return render_template('index.html', data_db=result)

async def read_db():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    
   # test_value = {
   #     "filename": "filename2",
   #     "content": "Some long string content"
   # }
   # await consumed_thing.write_property("someStringProperty", test_value)
    
   # LOGGER.info('Consumed Thing: {}'.format(consumed_thing))
    # Get the filename from the query parameters
    filename = request.args.get('filename')

    #result = await consumed_thing.read_property("someStringProperty")
  #  result = await consumed_thing.read_property(filename)
  #  LOGGER.info('Result after reading property is {}'.format(result))
    result = await consumed_thing.invoke_action("filenamesReadDB")
    return result


    
    
@app.route("/store_map_db", methods=['GET'])
def store_map_db():
    result = asyncio.run(storemapdb())
    return render_template('index.html', store_map_db=result)

async def storemapdb():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    # Get the filename from the query parameters
    filename_tosave = request.args.get('filename_tosave')
    result = await consumed_thing.invoke_action("mapStoreDB", {'filename_tosave': filename_tosave }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result)
    return result


@app.route("/store_bag_vo", methods=['GET'])
def store_bag_vo():
    result = asyncio.run(storebagvo())
    return render_template('index.html', store_bag_vo=result)

async def storebagvo():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    # Get the filename from the query parameters
    bagname_tosave = request.args.get('bagname_tosave')
    result = await consumed_thing.invoke_action("bagStoreVO", {'filename_tosave': bagname_tosave }) 
    #result = await consumed_thing.invoke_action("mapStoreDB")
    print(result)
    return result


@app.route("/read_map_from_db", methods=['GET'])
def read_map_from_db():
    result = asyncio.run(read_map_db())
    return render_template('index.html', map_from_db=result)


async def read_map_db():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1")
    

    # Get the filename from the query parameters
    filename_map = request.args.get('filename_map')

    #result = await consumed_thing.read_property("someStringProperty")
    result = await consumed_thing.invoke_action("mapReadDB", {'filename_map': filename_map })
    if result is None:
        return None
    else:
        pgm_raw_data= base64.b64decode(result)
    # Open PGM data as an image
        image_path_db = '/app/image_map_db.png'
        with Image.open(BytesIO(pgm_raw_data)) as img:
        # Convert to PNG format
            img.save(image_path_db, format="PNG")
        return image_path_db


if __name__ == "__main__":
    app.run(debug=True)

