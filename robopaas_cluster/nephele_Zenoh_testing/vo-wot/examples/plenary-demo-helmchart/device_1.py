import random

LOWER_VALUE = 30
UPPER_VALUE = 50

async def temperature_read_handler():
    return random.randint(LOWER_VALUE, UPPER_VALUE)

humidity_read_handler = temperature_read_handler

async def currentValues_handler(params):
    return {
        'result': True,
        'message': {
            "temperature": random.randint(LOWER_VALUE,UPPER_VALUE),
            "humidity": random.randint(LOWER_VALUE,UPPER_VALUE)
        }
    }

temperature_init = LOWER_VALUE
