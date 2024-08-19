#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Request handler for Action interactions.
"""

from tornado.web import RequestHandler

import wotpy.protocols.http.handlers.utils as handler_utils


# noinspection PyAbstractClass,PyAttributeOutsideInit
class ActionInvokeHandler(RequestHandler):
    """Handler for Action invocation requests."""

    # noinspection PyMethodOverriding
    def initialize(self, http_server):
        self._server = http_server

    async def post(self, thing_name, name):
        """Invokes the action and returns the invocation result."""

        exposed_thing = handler_utils.get_exposed_thing(self._server, thing_name)
        valid_creds = await self._server._check_credentials(exposed_thing.title, self.request)
        if not valid_creds:
            handler_utils.request_auth(self, self._server.security_scheme, thing_name)
        else:
            input_value = handler_utils.get_argument(self, "input")
            try:
                result = await exposed_thing.actions[name].invoke(input_value)
                self.write({"result": result})
            except Exception as ex:
                self.write({"error": str(ex)})
