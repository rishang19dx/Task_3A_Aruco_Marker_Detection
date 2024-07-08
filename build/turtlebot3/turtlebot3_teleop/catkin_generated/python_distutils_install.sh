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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/rishang19_dx/task3_aruco/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/rishang19_dx/task3_aruco/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/rishang19_dx/task3_aruco/install/lib/python3/dist-packages:/home/rishang19_dx/task3_aruco/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rishang19_dx/task3_aruco/build" \
    "/usr/bin/python3" \
    "/home/rishang19_dx/task3_aruco/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/rishang19_dx/task3_aruco/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/rishang19_dx/task3_aruco/install" --install-scripts="/home/rishang19_dx/task3_aruco/install/bin"
