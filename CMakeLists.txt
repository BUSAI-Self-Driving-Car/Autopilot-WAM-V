CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Autopilot)

# We use additional modules that cmake needs to know about
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules ${CMAKE_MODULE_PATH})

# Use the enhanced messaging system
SET(NUCLEAR_ENHANCED_MESSAGES ON CACHE BOOL "Use enhanced messaging system" FORCE)

# Compiler Settings
INCLUDE(Compiler)

# Shared libraries used by all modules
INCLUDE(SharedLibraries)

# Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)
