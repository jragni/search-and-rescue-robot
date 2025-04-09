#!/bin/bash

BASE_PATH="/search-and-rescue-robot/config"
export ZENOH_SESSION_CONFIG_URI=${BASE_PATH}/sessionconfig.json5
export ZENOH_CLIENT_CONFIG_URI=${BASE_PATH}/clientconfig.json5
export ZENOH_ROUTER_CHECK_ATTEMPT=-1