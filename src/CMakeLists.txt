#############################
# CMake - Serial Bridge - root/src
#############################
find_package(amd_std)
find_package(tinyxml2 REQUIRED)
find_package(Boost 1.71 COMPONENTS thread system program_options filesystem REQUIRED)
set(SERIAL_BRIDGE_MODULE_SOURCES SerialBridgeMain.cpp)

add_executable(amm_serial_bridge ${SERIAL_BRIDGE_MODULE_SOURCES})

target_link_libraries(
   amm_serial_bridge
   PUBLIC amm_std
   PUBLIC Boost::thread
   PUBLIC Boost::system
   PUBLIC Boost::program_options
   PUBLIC Boost::filesystem
   PUBLIC pthread
   PUBLIC tinyxml2::tinyxml2
   ${TinyXML2_LIBRARIES}
)

install(TARGETS amm_serial_bridge RUNTIME DESTINATION bin)
install(DIRECTORY ../config DESTINATION bin)
