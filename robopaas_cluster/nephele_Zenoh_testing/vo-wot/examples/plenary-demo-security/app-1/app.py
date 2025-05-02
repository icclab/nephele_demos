#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import logging

from flask import Flask

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT
from wotpy.protocols.http.client import HTTPClient

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

app = Flask(__name__)

SECURITY_SCHEME_DICT = {
    "scheme": "oidc4vp"
}
CREDENTIALS_DICT = {
    "holder_url": "http://localhost:8085/auth",
    "requester": "app-1"
}

@app.route("/temperature")
def temperature():
    result = asyncio.run(temp())
    return f"<p>{result}</p>"

@app.route("/humidity")
def humidity():
    result = asyncio.run(hum())
    return f"<p>{result}</p>"

async def temp():
    http_client = HTTPClient()
    http_client.set_security(SECURITY_SCHEME_DICT, CREDENTIALS_DICT)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1", credentials_dict=CREDENTIALS_DICT)
    result = await consumed_thing.read_property("temperature")
    print(result)
    return result

async def hum():
    http_client = HTTPClient()
    http_client.set_security(SECURITY_SCHEME_DICT, CREDENTIALS_DICT)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url("http://vo1:9090/vo1", credentials_dict=CREDENTIALS_DICT)
    result = await consumed_thing.read_property("humidity")
    print(result)
    return result

if __name__ == "__main__":
    app.run(debug=True)
