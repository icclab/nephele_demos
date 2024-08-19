#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import asyncio
from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT
from wotpy.protocols.http.client import HTTPClient
async def main():
    http_client = HTTPClient()
    security_scheme_dict = {
        "scheme": "bearer"
    }
    credentials_dict = {
        "token": "token"
    }
    http_client.set_security(security_scheme_dict, credentials_dict)
    wot = WoT(servient=Servient(clients=[http_client]))
    consumed_thing = await wot.consume_from_url('http://vo1:9090/vo1')
    result = await consumed_thing.invoke_action('averageData')
    print(result)
    consumed_thing = await wot.consume_from_url('http://cvo:9090/cvo')
    for _ in range(5):
        result = await consumed_thing.properties["temperature"].read()
    result = await consumed_thing.invoke_action('forecast_data')
    print("forecast", result)

asyncio.run(main())
