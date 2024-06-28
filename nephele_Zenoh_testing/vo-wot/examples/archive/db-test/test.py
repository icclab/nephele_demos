#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import json
import logging
import os
import random
from wotpy.protocols.http.server import HTTPServer
from wotpy.protocols.coap.server import CoAPServer
from wotpy.protocols.ws.server import WebsocketServer
from wotpy.wot.servient import Servient
from wotpy.functions.functions import forecasting, mean_value, vo_status, device_status

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

TD = {
    'title': 'Test',
    'id': 'urn:dev:wot:test:test',
    'description': '''A test example.''',
    'securityDefinitions': {
        'nosec_sc': {
            'scheme': 'nosec'
        }
    },
    'security': 'nosec_sc',
    '@context': [
        'https://www.w3.org/2022/wot/td/v1.1',
    ],
    'properties': {
        'dummy': {
            'type': 'integer'
        },
        'nested': {
            'type': 'object',
            'first_prop': {
                'type': 'integer'
            },
            'second_prop':{
                'type': 'integer'
            }
        }
    },
    'actions': {
        'return_hello': {
            'description': 'Returns hello'
        }
    },
    'events': {
        'VO_Connection_Error': {
            'description': '''VO_Connection_Error. Emitted when the Connection to the VO fails.''',
            'data': {
                'type': 'boolean',
            },
        },
        'Device_Connection_Error': {
            'description': '''Device_Connection_Error. Emitted when the Connection to the Exposed Thing of the Device fails.''',
            'data': {
                'type': 'boolean',
            },
        },
    }
}

async def main():
    hostname = os.environ.get('HOSTNAME', None)
    influxdb_url = os.environ.get('INFLUXDB_URL', None)

    LOGGER.info('Creating HTTP server')
    http_server = HTTPServer()

    LOGGER.info('Creating CoAP server')
    coap_server = CoAPServer()

    LOGGER.info('Creating Websocket server')
    ws_server = WebsocketServer()

    LOGGER.info('Creating servient with TD catalogue')
    servient = Servient(
        hostname=hostname, influxdb_enabled=True,
        influxdb_url=influxdb_url, influxdb_token="my-token"
    )
    servient.add_server(http_server)
    servient.add_server(coap_server)
    servient.add_server(ws_server)

    LOGGER.info('Starting servient')
    wot = await servient.start()

    LOGGER.info('Exposing and configuring Thing')

    # Produce the Thing from Thing Description
    exposed_thing = wot.produce(json.dumps(TD))

    # Initialize the property value
    await exposed_thing.properties['dummy'].write(random.uniform(1,5))
    await exposed_thing.properties['dummy'].write(random.uniform(1,5))
    await exposed_thing.properties['dummy'].write(random.uniform(1,5))

    await exposed_thing.properties['nested'].write({
        'first_prop': 5,
        'second_prop': 25
    })

    async def hello(params):
        return "hello"

    exposed_thing.set_action_handler("return_hello", hello)

    exposed_thing.expose()

    LOGGER.info('URL of Thing is at: {}'.format(exposed_thing.thing.url_name))

    LOGGER.info(f'{TD["title"]} is ready')

    # forecasted_data = await forecasting(wot, 'dummy')
    # print ("Forecasted data:", forecasted_data)
    # average = await mean_value(wot, 'dummy', 2)
    # print (average)
    temp = await vo_status(exposed_thing, 1)
    temp = await device_status(exposed_thing, 'http://localhost:9090', 1)


if __name__ == '__main__':
    LOGGER.info('Starting loop')
    loop = asyncio.new_event_loop()
    loop.create_task(main())
    loop.run_forever()
