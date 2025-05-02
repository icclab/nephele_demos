#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio

import requests
from flask import Flask

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT
from wotpy.protocols.http.client import HTTPClient

app = Flask(__name__)

@app.route("/fire")
def fire_detection():
    url = "http://cvo:8080/cvo/property/fire"
    result = requests.get(url)
    fire = "Fire" if result.json()["value"] else "Normal"
    print(fire)
    return f"<p>{fire}</p>"


@app.route("/average_data")
def average_data():
    result = asyncio.run(average())
    print("average", result)
    return f"<p>{result}</p>"

@app.route("/forecast_data")
def forecast_data():
    result = asyncio.run(forecast())
    print("forecast", result)
    return f"<p>{result}</p>"


async def average():
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
    result = await consumed_thing.invoke_action("averageData")
    print(result)
    return result

async def forecast():
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
    result = await consumed_thing.invoke_action("averageData")
    print(result)
    consumed_thing = await wot.consume_from_url("http://cvo:9090/cvo")
    for _ in range(5):
        result = await consumed_thing.properties["temperature"].read()
    result = await consumed_thing.invoke_action("forecast_data")
    return result

if __name__ == "__main__":
    app.run(debug=True)
