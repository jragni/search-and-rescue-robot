#!/bin/bash

BASE_PATH="/search-and-rescue-robot/config"
export ZENOH_SESSION_CONFIG_URI=$HOME/search-and-rescue-robot/config/sessionconfig.json5
export ZENOH_CLIENT_CONFIG_URI=$HOME/search-and-rescue-robot/config/clientconfig.json5
export ZENOH_ROUTER_CHECK_ATTEMPT=-1