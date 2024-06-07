import json
import logging
import math

LOGGER = logging.getLogger()

def read_from_sensor(sensorType):
    # Actual implementation of reading data from a sensor can go here
    # For the sake of example, let's just return a value
    return 100

def notify(msg, subscribers=['admin@coffeeMachine.com']):
    # Actual implementation of notifying subscribers with a message can go here
    LOGGER.info(msg)

allAvailableResources_init = {
    'water': read_from_sensor('water'),
    'milk': read_from_sensor('milk'),
    'chocolate': read_from_sensor('chocolate'),
    'coffeeBeans': read_from_sensor('coffeeBeans'),
}
possibleDrinks_init = ['espresso', 'americano', 'cappuccino', 'latte', 'hotChocolate', 'hotWater']
maintenanceNeeded_init = False
schedules_init = []

def maintenanceNeeded_on_next(data):
    notify(f'Value changed for an observable property: {data}')

def maintenanceNeeded_on_completed():
    notify('Subscribed for an observable property: maintenanceNeeded')

def maintenanceNeeded_on_error(error):
    notify(f'Error for an observable property maintenanceNeeded: {error}')

async def servedCounter_write_handler(value):
    await exposed_thing._default_update_property_handler('servedCounter', value)

    if value > 1000:
        await exposed_thing.properties['maintenanceNeeded'].write(True)

servedCounter_init = read_from_sensor('servedCounter')

async def makeDrink_handler(params):
    params = params['input'] if params['input'] else {}

    # Default values
    drinkId = 'americano'
    size = 'm'
    quantity = 1

    # Size quantifiers
    sizeQuantifiers = {'s': 0.1, 'm': 0.2, 'l': 0.3}

    # Drink recipes showing the amount of a resource consumed for a particular drink
    drinkRecipes = {
        'espresso': {
            'water': 1,
            'milk': 0,
            'chocolate': 0,
            'coffeeBeans': 2,
        },
        'americano': {
            'water': 2,
            'milk': 0,
            'chocolate': 0,
            'coffeeBeans': 2,
        },
        'cappuccino': {
            'water': 1,
            'milk': 1,
            'chocolate': 0,
            'coffeeBeans': 2,
        },
        'latte': {
            'water': 1,
            'milk': 2,
            'chocolate': 0,
            'coffeeBeans': 2,
        },
        'hotChocolate': {
            'water': 0,
            'milk': 0,
            'chocolate': 1,
            'coffeeBeans': 0,
        },
        'hotWater': {
            'water': 1,
            'milk': 0,
            'chocolate': 0,
            'coffeeBeans': 0,
        },
    }

    # Check if params are provided
    drinkId = params.get('drinkId', drinkId)
    size = params.get('size', size)
    quantity = params.get('quantity', quantity)

    # Read the current level of allAvailableResources
    resources = await exposed_thing.read_property('allAvailableResources')

    # Calculate the new level of resources
    newResources = resources.copy()
    newResources['water'] -= math.ceil(quantity * sizeQuantifiers[size] * drinkRecipes[drinkId]['water'])
    newResources['milk'] -= math.ceil(quantity * sizeQuantifiers[size] * drinkRecipes[drinkId]['milk'])
    newResources['chocolate'] -= math.ceil(quantity * sizeQuantifiers[size] * drinkRecipes[drinkId]['chocolate'])
    newResources['coffeeBeans'] -= math.ceil(quantity * sizeQuantifiers[size] * drinkRecipes[drinkId]['coffeeBeans'])

    # Check if the amount of available resources is sufficient to make a drink
    for resource, value in newResources.items():
        if value <= 0:
            # Emit outOfResource event
            exposed_thing.emit_event('outOfResource', f'Low level of {resource}: {resources[resource]}%')
            return {'result': False, 'message': f'{resource} level is not sufficient'}

    # Now store the new level of allAvailableResources and servedCounter
    await exposed_thing.properties['allAvailableResources'].write(newResources)

    servedCounter = await exposed_thing.read_property('servedCounter')
    servedCounter += quantity
    await exposed_thing.properties['servedCounter'].write(servedCounter)

    # Finally deliver the drink
    return {'result': True, 'message': f'Your {drinkId} is in progress!'}

async def setSchedule_handler(params):
    params = params['input'] if params['input'] else {}

    # Check if required fields are provided in input
    if 'time' in params and 'mode' in params:

        # Use default values for non-required fields if not provided
        params['drinkId'] = params.get('drinkId', 'americano')
        params['size'] = params.get('size', 'm')
        params['quantity'] = params.get('quantity', 1)

        # Now read the schedules property, add a new schedule to it and then rewrite the schedules property
        schedules = await exposed_thing.read_property('schedules')
        schedules.append(params)
        await exposed_thing.properties['schedules'].write(schedules)
        return {'result': True, 'message': 'Your schedule has been set!'}

    return {'result': False, 'message': 'Please provide all the required parameters: time and mode.'}
