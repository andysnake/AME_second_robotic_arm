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

echo_and_run cd "/home/ying/Desktop/new_project547_cong2/src/universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ying/Desktop/new_project547_cong2/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ying/Desktop/new_project547_cong2/install/lib/python2.7/dist-packages:/home/ying/Desktop/new_project547_cong2/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ying/Desktop/new_project547_cong2/build" \
    "/usr/bin/python2" \
    "/home/ying/Desktop/new_project547_cong2/src/universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/ying/Desktop/new_project547_cong2/build/universal_robot/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ying/Desktop/new_project547_cong2/install" --install-scripts="/home/ying/Desktop/new_project547_cong2/install/bin"
