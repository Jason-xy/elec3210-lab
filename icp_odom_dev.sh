#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

export APP_PATH=$SCRIPT_DIR/icp_odom
export DATA_PATH=$SCRIPT_DIR/dataset

docker-compose up -d