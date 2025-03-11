#!/bin/bash

# Update these parameters as needed
WATCH_DIRS="./src ./include ./unit_tests ./executables"
BUILD_DIR=build
# EXECUTABLE="./build/executables/dmp/s3_test"
EXECUTABLE=""


START_BUILD_ICON=applications-system-symbolic
SUCCESS_ICON=emblem-ok-symbolic
FAILED_ICOND=emblem-important-symbolic

echo "Watching directories $WATCH_DIRS"
echo "Build directory: $BUILD_DIR"
echo ""

while true; do
    inotifywait -e close_write,modify,moved_to,create -r $WATCH_DIRS
    clear
    
    notify-send "Starting build proces..." -i $START_BUILD_ICON
    cmake --build $BUILD_DIR --parallel
    if [ $? -eq 0 ]; then
        notify-send "Build successful!" -i $SUCCESS_ICON
    else
        notify-send "Build failed!" -i $FAILED_ICOND
        continue
    fi

    GTEST_COLOR=1 ctest --test-dir $BUILD_DIR
    if [ $? -eq 0 ]; then
        notify-send "Tests passed!" -i $SUCCESS_ICON
    else
        notify-send "Some tests failed!" -i $FAILED_ICOND
        cat $BUILD_DIR/Testing/Temporary/LastTest.log
        continue
    fi

    if [ -z "$EXECUTABLE" ]; then
        echo "No executable provided"
    elif [ ! -x "$EXECUTABLE" ]; then
        echo "Executable $EXECUTABLE not found"
    else
        echo "Running $EXECUTABLE"
        $EXECUTABLE
    fi
done
