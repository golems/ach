#!/bin/sh -e

for DOCKERFILE in $@; do
      docker build -t "ach:$DOCKERFILE-dep" -f "script/docker/$DOCKERFILE" .;
done
