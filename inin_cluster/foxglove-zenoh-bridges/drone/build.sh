#!/usr/bin/env bash

################################################################################
# ZHAW INIT
# Description:  Shell script to create the Base GPU Docker image
# Authors:      Mark Straub
# Date:         2024-01-23
################################################################################
export IMAGE_NAME=robopaas/foxglove-zenoh-bridge-drone

# Get this script's path
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

# Build the docker image
docker build  \
=======
docker build \
  --build-arg user=user\
  --build-arg uid=$UID\
  --build-arg home=/home/user \
  --build-arg workspace=/home/user \
  --build-arg shell=$SHELL\
  -t $IMAGE_NAME .

