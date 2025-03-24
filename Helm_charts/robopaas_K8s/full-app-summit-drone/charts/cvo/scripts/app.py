#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import asyncio
#import mcap
from wotpy.functions.functions import vo_status, device_status

LOGGER = logging.getLogger()

from io import BytesIO
import base64
LOGGER.setLevel(logging.INFO)


async def read_property_from_drone():
    # Initialize the property values
    allAvailableResources_drone = await consumed_vos["vo1"].properties['allAvailableResources_drone'].read()
    possibleLaunchfiles_drone = await consumed_vos["vo1"].properties['possibleLaunchfiles_drone'].read()
    
    # Initialize the property values
    await exposed_thing.properties['allAvailableResources_drone'].write(allAvailableResources_drone)
    await exposed_thing.properties['possibleLaunchfiles_drone'].write(possibleLaunchfiles_drone)

async def read_property_from_summit():
    # Initialize the property values
    allAvailableResources_summit = await consumed_vos["vo2"].properties['allAvailableResources_summit'].read()
    possibleLaunchfiles_summit = await consumed_vos["vo2"].properties['possibleLaunchfiles_summit'].read()
    
    # Initialize the property values
    await exposed_thing.properties['allAvailableResources_summit'].write(allAvailableResources_summit)
    await exposed_thing.properties['possibleLaunchfiles_summit'].write(possibleLaunchfiles_summit)


