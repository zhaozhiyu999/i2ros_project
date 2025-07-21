set(libsocket_INCLUDE_DIRS "/home/youran/Desktop/i2ros_project/project/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/youran/Desktop/i2ros_project/project/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
