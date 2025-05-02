async def smoke_write_handler(value):
    print ("New Smoke Value:", value)
    await exposed_thing._default_update_property_handler("smoke", bool(int(value)))
    if value:
        exposed_thing.emit_event("smokeDetected", "Smoke detected")
