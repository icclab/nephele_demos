#!/usr/bin/env python
# -*- coding: utf-8 -*-

from wotpy.functions.functions import mean_value, vo_status, device_status

async def averageData_handler(params):
    time_horizon = 2
    average_temperature = await mean_value(exposed_thing, "temperature", time_horizon)
    average_humidity = await mean_value(exposed_thing, "humidity", time_horizon)
    result = {"average_temperature": average_temperature, "average_humidity": average_humidity}
    print(result)
    return result

async def check():
    curr_vo_status = await vo_status(exposed_thing, 1)
    curr_device_status = await device_status(exposed_thing, "http://192.168.49.1:9090", 1)

    print(f"Status VO: {curr_vo_status}")
    print(f"Status Device: {curr_device_status}")

async def read_property_from_device_1():
    temperature = await consumed_vos["device1"].properties["temperature"].read()
    humidity = await consumed_vos["device1"].properties["humidity"].read()
    await exposed_thing.properties["temperature"].write(temperature)
    await exposed_thing.properties["humidity"].write(humidity)
