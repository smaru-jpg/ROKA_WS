#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "metis-gtsam" for configuration "None"
set_property(TARGET metis-gtsam APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(metis-gtsam PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libmetis-gtsam.so"
  IMPORTED_SONAME_NONE "libmetis-gtsam.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS metis-gtsam )
list(APPEND _IMPORT_CHECK_FILES_FOR_metis-gtsam "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libmetis-gtsam.so" )

# Import target "CppUnitLite" for configuration "None"
set_property(TARGET CppUnitLite APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CppUnitLite PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libCppUnitLite.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS CppUnitLite )
list(APPEND _IMPORT_CHECK_FILES_FOR_CppUnitLite "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libCppUnitLite.a" )

# Import target "gtsam" for configuration "None"
set_property(TARGET gtsam APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(gtsam PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libgtsam.so.4.2.0"
  IMPORTED_SONAME_NONE "libgtsam.so.4"
  )

list(APPEND _IMPORT_CHECK_TARGETS gtsam )
list(APPEND _IMPORT_CHECK_FILES_FOR_gtsam "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libgtsam.so.4.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
