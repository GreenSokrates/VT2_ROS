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

echo_and_run cd "/home/luis/VT2/catkin_ws/src/universal_robot/ur_driver"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/luis/VT2/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/luis/VT2/catkin_ws/install/lib/python2.7/dist-packages:/home/luis/VT2/catkin_ws/build/ur_driver/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/luis/VT2/catkin_ws/build/ur_driver" \
    "/usr/bin/python" \
    "/home/luis/VT2/catkin_ws/src/universal_robot/ur_driver/setup.py" \
    build --build-base "/home/luis/VT2/catkin_ws/build/ur_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/luis/VT2/catkin_ws/install" --install-scripts="/home/luis/VT2/catkin_ws/install/bin"
