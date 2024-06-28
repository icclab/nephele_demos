#!/usr/bin/env python
# -*- coding: utf-8 -*-

from faker import Faker

from tests.utils import run_test_coroutine
from wotpy.protocols.http.client import HTTPClient
from wotpy.protocols.ws.client import WebsocketClient
from wotpy.support import is_coap_supported, is_mqtt_supported
from wotpy.wot.td import ThingDescription


def test_all_protocols_combined(all_protocols_servient):
    """Protocol bindings work as expected when multiple
    servers are combined within the same Servient."""

    exposed_thing = next(all_protocols_servient.exposed_things)
    td = ThingDescription.from_thing(exposed_thing.thing)

    clients = [
        WebsocketClient(),
        HTTPClient()
    ]

    if is_coap_supported():
        from wotpy.protocols.coap.client import CoAPClient
        clients.append(CoAPClient())

    if is_mqtt_supported():
        from tests.protocols.mqtt.broker import is_test_broker_online
        from wotpy.protocols.mqtt.client import MQTTClient
        if is_test_broker_online():
            clients.append(MQTTClient())

    prop_name = next(iter(td.properties.keys()))

    async def read_property(the_client):
        prop_value = Faker().sentence()

        curr_value = await the_client.read_property(td, prop_name)
        assert curr_value != prop_value

        await exposed_thing.properties[prop_name].write(prop_value)

        curr_value = await the_client.read_property(td, prop_name)
        assert curr_value == prop_value

    async def write_property(the_client):
        updated_value = Faker().sentence()

        curr_value = await exposed_thing.properties[prop_name].read()
        assert curr_value != updated_value

        await the_client.write_property(td, prop_name, updated_value)

        curr_value = await exposed_thing.properties[prop_name].read()
        assert curr_value == updated_value

    async def test_coroutine():
        for client in clients:
            await read_property(client)
            await write_property(client)

    run_test_coroutine(test_coroutine)
