import asyncio

from wotpy.functions.functions import forecasting

fire_init = False

async def fire_detection(data):
    if "Smoke detected" in str(data):
        smoke = True
    else:
        smoke = False
    await exposed_thing.properties["smoke"].write(smoke)
    results = await consumed_vos["vo1"].invoke_action("currentValues")
    temperature = results["message"]["temperature"]
    if temperature > 30 and smoke:
        exposed_thing.emit_event("fireDetected", "Fire detected")
        await exposed_thing.properties["fire"].write(True)

def smokeDetected_vo2_on_next(data):
    asyncio.create_task(fire_detection(data))

def smokeDetected_vo2_on_completed():
    print("Completed")

def smokeDetected_vo2_on_error(err):
    print(err)

async def forecast_data_handler(params):
    forecasted_data = await forecasting(exposed_thing, "temperature")
    print ("Forecasted data:", forecasted_data)
    return {'result': True, 'message': {"forecasted_data": forecasted_data}}
