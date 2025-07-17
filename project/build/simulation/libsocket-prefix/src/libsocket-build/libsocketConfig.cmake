set(libsocket_INCLUDE_DIRS "/home/youran/Documents/introtoros_2025/project/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/youran/Documents/introtoros_2025/project/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
