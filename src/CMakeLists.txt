#############################
# CMake - Serial Bridge - root/src
#############################

set(SERIAL_BRIDGE_MODULE_SOURCES SerialBridgeMain.cpp)

add_executable(amm_serial_bridge ${SERIAL_BRIDGE_MODULE_SOURCES})

target_link_libraries(
   amm_serial_bridge
   PUBLIC amm_std
   PUBLIC Boost::thread
   PUBLIC Boost::system
   PUBLIC pthread
   tinyxml2
   PUBLIC gpiod
)

install(TARGETS amm_serial_bridge RUNTIME DESTINATION bin)
install(DIRECTORY ../config DESTINATION bin)
