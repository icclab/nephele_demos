#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import logging

from wotpy.wot.servient import Servient
from wotpy.wot.wot import WoT

logging.basicConfig()
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.INFO)


async def main():
    wot = WoT(servient=Servient())
    consumed_thing = await wot.consume_from_url('http://127.0.0.1:9090/testvo')

    LOGGER.info('Consumed Thing: {}'.format(consumed_thing))

    await consumed_thing.write_property('dummy', 1500)
    value = await consumed_thing.read_property('dummy')
    print(value)


if __name__ == '__main__':
    loop = asyncio.run(main())
