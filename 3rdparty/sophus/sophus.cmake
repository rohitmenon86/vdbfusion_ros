include(ExternalProject)

ExternalProject_Add(
  external_sophus
  PREFIX sophus
  URL https://github.com/strasdat/Sophus/archive/refs/tags/v22.04.1.tar.gz
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             -DSOPHUS_USE_BASIC_LOGGING=ON 
             -DBUILD_SOPHUS_EXAMPLES=OFF
             -DBUILD_SOPHUS_TESTS=OFF 
             -DCMAKE_BUILD_TYPE=Release)

ExternalProject_Get_Property(external_sophus SOURCE_DIR)
add_library(SophusHelper INTERFACE)
add_dependencies(SophusHelper external_sophus)
target_compile_definitions(SophusHelper INTERFACE SOPHUS_USE_BASIC_LOGGING=1)
target_include_directories(SophusHelper INTERFACE ${SOURCE_DIR}/)
set_property(TARGET SophusHelper PROPERTY EXPORT_NAME sophus::sophus)
add_library(sophus::sophus ALIAS SophusHelper)