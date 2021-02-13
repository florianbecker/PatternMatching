# PatternMatching
Pattern matching with pylon SDK and OpenCV on embedded Jetson Nano

## User-defined build variables
Inside cmake/env.cmake you will find some user-defined build variables for particular purposes.

### Debugging
You can log the verbose build output.
```cmake
# Debugging of build steps
set(CMAKE_VERBOSE_MAKEFILE ON)
```

### Qt6
You can switch from Qt5 to Qt6 - default: Qt5
```cmake
# Enable Qt build - use Qt5 or Qt6
set(QtVERSION Qt6)
```
