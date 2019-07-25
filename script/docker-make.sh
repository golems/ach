#!/bin/sh -e

if [ -z "$JOBS" ]; then
    JOBS=8
fi

for DOCKERFILE in $@; do
    NAME="ach-${DOCKERFILE}-install"
    docker rm "$NAME" || true
    docker run --name="$NAME" "ach:$DOCKERFILE-dep" /bin/sh -c \
         "cd /root/ach && autoreconf -i && ./configure --disable-kbuild --disable-dkms --disable-dkms-build --with-java && make  -k -j $JOBS V=1 && make -k -j $JOBS V=1 distcheck && make install"

            #&&  docker commit "$NAME" "ach:${DOCKERFILE}-install"
done
