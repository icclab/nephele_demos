#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import uuid

import pytest
from faker import Faker

from tests.utils import find_free_port
from wotpy.protocols.http.server import HTTPServer
from wotpy.wot.constants import WOT_TD_CONTEXT_URL_V1_1
from wotpy.wot.dictionaries.interaction import PropertyFragmentDict, ActionFragmentDict, EventFragmentDict
from wotpy.wot.dictionaries.thing import ThingFragment
from wotpy.wot.exposed.thing import ExposedThing
from wotpy.wot.servient import Servient
from wotpy.wot.td import ThingDescription
from wotpy.wot.thing import Thing


@pytest.fixture
def http_server():
    """Builds an HTTPServer instance that contains an ExposedThing."""

    port = find_free_port()

    thing_fragment = ThingFragment({
        "@context": [
            WOT_TD_CONTEXT_URL_V1_1,
        ],
        "id": uuid.uuid4().urn,
        "title": uuid.uuid4().hex,
        "securityDefinitions": {
            "nosec_sc":{
                "scheme":"nosec" 
            }
        },
        "security": "nosec_sc"
    })
    thing = Thing(thing_fragment=thing_fragment)
    exposed_thing = ExposedThing(servient=Servient(), thing=thing)

    property_name_01 = uuid.uuid4().hex
    exposed_thing.add_property(property_name_01, PropertyFragmentDict({
        "type": "number",
        "observable": True
    }), value=Faker().pyint())

    property_name_02 = uuid.uuid4().hex
    exposed_thing.add_property(property_name_02, PropertyFragmentDict({
        "type": "number",
        "observable": True
    }), value=Faker().pyint())

    event_name = uuid.uuid4().hex
    exposed_thing.add_event(event_name, EventFragmentDict({
        "type": "object"
    }))

    action_name = uuid.uuid4().hex

    async def triple(parameters):
        input_value = parameters.get("input")
        await asyncio.sleep(0)
        return(input_value * 3)

    exposed_thing.add_action(action_name, ActionFragmentDict({
        "input": {"type": "number"},
        "output": {"type": "number"}
    }), triple)

    server = HTTPServer(port=port)
    server.add_exposed_thing(exposed_thing)

    async def start():
        await server.start()

    loop = asyncio.get_event_loop_policy().get_event_loop()
    wot = loop.run_until_complete(start())

    yield server

    async def stop():
        await server.stop()

    loop.run_until_complete(stop())


@pytest.fixture
def http_servient():
    """Returns a Servient that exposes an HTTP server and one ExposedThing."""

    http_port = find_free_port()
    http_server = HTTPServer(port=http_port)

    servient = Servient(catalogue_port=None)
    servient.add_server(http_server)

    async def start():
        return(await servient.start())

    loop = asyncio.get_event_loop_policy().get_event_loop()
    wot = loop.run_until_complete(start())

    property_name_01 = uuid.uuid4().hex
    property_name_02 = uuid.uuid4().hex
    action_name_01 = uuid.uuid4().hex
    event_name_01 = uuid.uuid4().hex

    td_dict = {
        "@context": [
            WOT_TD_CONTEXT_URL_V1_1,
        ],
        "id": uuid.uuid4().urn,
        "title": uuid.uuid4().hex,
        "securityDefinitions": {
            "nosec_sc":{
                "scheme":"nosec" 
            }
        },
        "security": "nosec_sc",
        "properties": {
            property_name_01: {
                "observable": True,
                "type": "string"
            },
            property_name_02: {
                "observable": True,
                "type": "string"            
            }
        },
        "actions": {
            action_name_01: {
                "input": {
                    "type": "number"
                },
                "output": {
                    "type": "number"
                }                
            }
        },
        "events": {
            event_name_01: {
                "type": "string",                     
            }
        },
    }

    td = ThingDescription(td_dict)

    exposed_thing = wot.produce(td.to_str())
    exposed_thing.expose()

    async def action_handler(parameters):
        input_value = parameters.get("input")
        return(int(input_value) * 2)

    exposed_thing.set_action_handler(action_name_01, action_handler)

    yield servient

    async def shutdown():
        await servient.shutdown()

    loop.run_until_complete(shutdown())
