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

echo_and_run cd "/home/liding/ws_thesis_lzq/src/isaac_ros_navigation_goal"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/liding/ws_thesis_lzq/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/liding/ws_thesis_lzq/install/lib/python3/dist-packages:/home/liding/ws_thesis_lzq/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/liding/ws_thesis_lzq/build" \
    "/usr/bin/python3" \
    "/home/liding/ws_thesis_lzq/src/isaac_ros_navigation_goal/setup.py" \
    egg_info --egg-base /home/liding/ws_thesis_lzq/build/isaac_ros_navigation_goal \
    build --build-base "/home/liding/ws_thesis_lzq/build/isaac_ros_navigation_goal" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/liding/ws_thesis_lzq/install" --install-scripts="/home/liding/ws_thesis_lzq/install/bin"
