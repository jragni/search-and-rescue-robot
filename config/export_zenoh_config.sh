#!/bin/bash

export ZENOH_SESSION_CONFIG_URI=$HOME/search-and-rescue-robot/config/sessionconfig.json5
export ZENOH_ROUTER_CONFIG_URI=$HOME/search-and-rescue-robot/config/routerconfig.json5
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE="scouting/multicast/enabled=true"
export ROS_DOMAIN_ID=30