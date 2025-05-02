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
    
    LOGGER.info('Consumed Thing: {}'.format(consumed_thing))

    test_value = {
        "name": "test-name",
        "content": "Some long string content"
    }
    await consumed_thing.write_property("someStringProperty", test_value)
    result = await consumed_thing.read_property("someStringProperty")
    LOGGER.info('Result after writing and reading property is {}'.format(result))

    #result = await consumed_thing.properties["allAvailableResources"].read()
    
    #result = await consumed_thing.invoke_action("currentValues")
    print(result)
    return result

if __name__ == "__main__":
    app.run(debug=True)


