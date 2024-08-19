#!/bin/ash
cat /entrypoint.sh
echo " * Starting custom: /$BINARY $*"
exec /$BINARY $*
