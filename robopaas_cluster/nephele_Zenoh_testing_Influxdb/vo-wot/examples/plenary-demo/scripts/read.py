#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import logging
import json
import aiocoap
from aiocoap import *

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)

async def main():
    protocol = await Context.create_client_context()
    VO_2_url = "192.168.49.2:30012"
    request = Message(code=aiocoap.Code.GET, uri=f'coap://{VO_2_url}/property?thing=vo2&name=smoke', )
    try:
        response = await protocol.request(request).response
    except Exception as e:
        print('Failed to fetch resource:')
        print(e)
    else:
        print('Result: %s\n%r'%(response.code, response.payload))

if __name__ == '__main__':
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())

