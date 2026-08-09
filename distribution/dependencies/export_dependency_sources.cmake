cmake_minimum_required(VERSION 3.24)

foreach(required_variable IN ITEMS
        ORVD_DEPENDENCY_SOURCE_MODULE
        ORVD_DEPENDENCY_SOURCE_MANIFEST
        ORVD_DEPENDENCY_EXPORT_DIRECTORY)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} must be provided")
    endif()
endforeach()

include("${ORVD_DEPENDENCY_SOURCE_MODULE}")
orvd_begin_dependency_sources()
include("${ORVD_DEPENDENCY_SOURCE_MANIFEST}")
orvd_finalize_dependency_sources()
orvd_export_dependency_sources("${ORVD_DEPENDENCY_EXPORT_DIRECTORY}")
