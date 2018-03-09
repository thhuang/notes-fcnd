#!/bin/sh

PRJ_ROOT='/data/thhuang/fcnd'
INPUT_PATH='/data/thhuang/fcnd_input'
OUTPUT_PATH='/data/thhuang/fcnd_output'
IMAGE_NAME='thhuang/fcnd'
PORT='8642'

nvidia-docker run -ti -p ${PORT}:8888 \
                  -v ${PRJ_ROOT}:/app \
                  -v ${INPUT_PATH}:/app/data/input:ro \
                  -v ${OUTPUT_PATH}:/app/data/output \
                  ${IMAGE_NAME} bash

