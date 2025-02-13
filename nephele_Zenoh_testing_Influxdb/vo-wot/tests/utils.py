#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import os
import socket

from tornado.escape import to_unicode

DEFAULT_TIMEOUT_SECS = 30
TIMEOUT_CORO_VAR = "WOTPY_TESTS_CORO_TIMEOUT"


def run_test_coroutine(coro, timeout=None):
    """Synchronously runs the given test coroutine with an optinally defined timeout."""

    timeout = timeout if timeout else os.getenv(TIMEOUT_CORO_VAR, str(DEFAULT_TIMEOUT_SECS))

    async def main():
        await asyncio.wait_for(
            coro(), timeout=float(timeout))

    loop = asyncio.get_event_loop_policy().get_event_loop()
    loop.run_until_complete(main())

def assert_equal_dict(dict_a, dict_b, compare_as_unicode=False):
    """Asserts that both dicts are equal."""

    assert set(dict_a.keys()) == set(dict_b.keys())

    for key in dict_a:
        value_a = dict_a[key]
        value_b = dict_b[key]

        if compare_as_unicode and isinstance(value_a, str):
            assert to_unicode(value_a) == to_unicode(value_b)
        else:
            assert value_a == value_b


def find_free_port():
    """Returns a free TCP port by attempting to open a socket on an OS-assigned port."""

    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))
        return sock.getsockname()[1]
    finally:
        if sock:
            sock.close()
