#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""MQTT Client CLI - Entrypoint.

This file serves as the entrypoint for the MQTT client CLI.
All functionality is implemented in the mqtt_client package.
"""

import os

from mqtt_client import main, interactive_shell

if __name__ == "__main__":
    if os.environ.get("MQTT_INTERACTIVE"):
        interactive_shell()
    else:
        main()
