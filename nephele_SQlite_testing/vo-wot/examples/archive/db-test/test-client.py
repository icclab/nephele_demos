#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import asyncio
import logging

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT
from wotpy.protocols.http.client import HTTPClient
logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)


async def main():
    http_client = HTTPClient()
    wot = WoT(servient=Servient(clients=[http_client]))

    LOGGER.info('Clients: {}'.format(wot.servient.clients))
    consumed_thing = await wot.consume_from_url('http://127.0.0.1:9090/test')

    LOGGER.info('Consumed Thing: {}'.format(consumed_thing))
    await consumed_thing.write_property('dummy', [4, 5, 6])
    result = await consumed_thing.read_property('dummy')
    print(result)
    await consumed_thing.write_property('nested', {'first_prop':2,'second_prop':3})
    result = await consumed_thing.read_property('nested')
    print(result)
    result = await consumed_thing.invoke_action("return_hello")
    print(result)



if __name__ == '__main__':
    asyncio.run(main())
