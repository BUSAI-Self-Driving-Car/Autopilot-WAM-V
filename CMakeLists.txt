CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Autopilot)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/nuclear/cmake/Modules ${CMAKE_MODULE_PATH})

# Use the UoN RobotX Banner
SET(NUCLEAR_ROLE_BANNER_FILE "${CMAKE_SOURCE_DIR}/cmake/banner.txt" CACHE PATH "The path the banner to print at the start of each role execution" FORCE)

# Use the enhanced messaging system
SET(NUCLEAR_ENHANCED_MESSAGES ON CACHE BOOL "Use enhanced messaging system" FORCE)

# Compiler Settings
INCLUDE(Compiler)

# Shared libraries used by all modules
INCLUDE(SharedLibraries)

# Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)
