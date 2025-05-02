#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import logging
import uuid

import pytest
from faker import Faker

from tests.utils import find_free_port
from wotpy.support import is_coap_supported
from wotpy.wot.constants import WOT_TD_CONTEXT_URL_V1_1
from wotpy.wot.dictionaries.interaction import PropertyFragmentDict, EventFragmentDict, ActionFragmentDict
from wotpy.wot.dictionaries.thing import ThingFragment
from wotpy.wot.exposed.thing import ExposedThing
from wotpy.wot.servient import Servient
from wotpy.wot.td import ThingDescription
from wotpy.wot.thing import Thing

collect_ignore = []

if not is_coap_supported():
    logging.warning("Skipping CoAP tests due to unsupported platform")
    collect_ignore += ["test_server.py", "test_client.py"]


@pytest.fixture(params=[{"action_clear_ms": 5000}])
def coap_server(request):
    """Builds a CoAPServer instance that contains an ExposedThing."""

    from wotpy.protocols.coap.server import CoAPServer

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
        "type": "string",
        "observable": True
    }), value=Faker().pyint())

    event_name = uuid.uuid4().hex
    exposed_thing.add_event(event_name, EventFragmentDict({
        "type": "object"
    }))

    action_name = uuid.uuid4().hex

    async def triple(parameters):
        input_value = parameters.get("input")
        return input_value * 3

    exposed_thing.add_action(action_name, ActionFragmentDict({
        "input": {"type": "number"},
        "output": {"type": "number"}
    }), triple)


    server = CoAPServer(port=port, **request.param)
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
def coap_servient():
    """Returns a Servient that exposes a CoAP server and one ExposedThing."""

    from wotpy.protocols.coap.server import CoAPServer

    coap_port = find_free_port()
    the_coap_server = CoAPServer(port=coap_port)

    servient = Servient(catalogue_port=None)
    servient.add_server(the_coap_server)

    async def start():
        return(await servient.start())

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    wot = loop.run_until_complete(start())

    property_name = uuid.uuid4().hex
    action_name = uuid.uuid4().hex
    event_name = uuid.uuid4().hex

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
            property_name: {
                "observable": True,
                "type": "string"
            }
        },
        "actions": {
            action_name: {
                "input": {
                    "type": "number"
                },
                "output": {
                    "type": "number"
                }
            }
        },
        "events": {
            event_name: {
                "type": "string"
            }
        }
    }

    td = ThingDescription(td_dict)

    exposed_thing = wot.produce(td.to_str())
    exposed_thing.expose()

    async def action_handler(parameters):
        input_value = parameters.get("input")
        return int(input_value) * 2

    exposed_thing.set_action_handler(action_name, action_handler)

    yield servient

    async def shutdown():
        await servient.shutdown()

    loop.run_until_complete(shutdown())
