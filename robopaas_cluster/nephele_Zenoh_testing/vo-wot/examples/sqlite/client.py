#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import asyncio
import logging

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)


async def main():
    wot = WoT(servient=Servient())
    consumed_thing = await wot.consume_from_url('http://127.0.0.1:9090/db-test')

    LOGGER.info('Consumed Thing: {}'.format(consumed_thing))

    test_value = {
        "name": "test-name",
        "content": "Some long string content"
    }
    await consumed_thing.write_property("someStringProperty", test_value)
    result = await consumed_thing.read_property("someStringProperty")
    LOGGER.info('Result after writing and reading property is {}'.format(result))

if __name__ == '__main__':
    loop = asyncio.run(main())
