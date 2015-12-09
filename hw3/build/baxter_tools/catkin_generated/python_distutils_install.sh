#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/matt/Documents/ros_sandbox/hw3/src/baxter_tools"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/matt/Documents/ros_sandbox/hw3/install/lib/python2.7/dist-packages:/home/matt/Documents/ros_sandbox/hw3/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/matt/Documents/ros_sandbox/hw3/build" \
    "/usr/bin/python" \
    "/home/matt/Documents/ros_sandbox/hw3/src/baxter_tools/setup.py" \
    build --build-base "/home/matt/Documents/ros_sandbox/hw3/build/baxter_tools" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/matt/Documents/ros_sandbox/hw3/install" --install-scripts="/home/matt/Documents/ros_sandbox/hw3/install/bin"
