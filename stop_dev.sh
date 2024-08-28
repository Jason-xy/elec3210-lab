#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

export APP_PATH=$SCRIPT_DIR
export DATA_PATH=$SCRIPT_DIR

docker-compose down