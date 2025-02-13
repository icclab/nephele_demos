#!/usr/bin/env python
# -*- coding: utf-8 -*-

from wotpy.wot.constants import WOT_TD_CONTEXT_URL_V1_1

TD_EXAMPLE = {
    "@context": [
        WOT_TD_CONTEXT_URL_V1_1
    ],
    "id": "urn:dev:wot:com:example:servient:lamp",
    "title": "MyLampThing",
    "description": "MyLampThing uses JSON-LD 1.1 serialization",
    "security": ["nosec_sc"],
    "securityDefinitions": {
        "nosec_sc": {"scheme": "nosec"}
    },
    "properties": {
        "status": {
            "description": "Shows the current status of the lamp",
            "type": "string",
            "forms": [{
                "href": "coaps://mylamp.example.com/status"
            }]
        }
    },
    "actions": {
        "toggle": {
            "description": "Turn on or off the lamp",
            "forms": [{
                "href": "coaps://mylamp.example.com/toggle"
            }]
        }
    },
    "events": {
        "overheating": {
            "description": "Lamp reaches a critical temperature (overheating)",
            "data": {"type": "string"},
            "forms": [{
                "href": "coaps://mylamp.example.com/oh"
            }]
        }
    }
}
