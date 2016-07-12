# We need noncall exceptions so we can throw exceptions from signal handlers
# This allows us to catch null pointer exceptions
SET(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -march=native -mtune=native -fnon-call-exceptions -pthread")

# GNU Compiler
IF(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    # Enable colours on g++ 4.9 or greater
    IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
    ENDIF()
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS}")

# Do special things on a GNU compiler
IF("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")

    # If we have g++4.9 or later, then enable coloured output
    IF(GCC_VERSION VERSION_GREATER "4.9" OR GCC_VERSION VERSION_EQUAL "4.9")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
    ENDIF()
ENDIF()
