include(CMakePrintHelpers)

set(srcs
        "ArduinoBluepad32.cpp"
        "ArduinoConsole.cpp"
        "ArduinoController.cpp"
        "ArduinoGamepad.cpp"
        "arduino_bootstrap.cpp"
        "arduino_platform.c")

# Changed for ieee-balance-bot: PlatformIO's hybrid build (framework = arduino, espidf)
# names the Arduino core component framework-arduinoespressif32, not arduino.
idf_build_get_property(_component_targets __COMPONENT_TARGETS)
if("___idf_framework-arduinoespressif32" IN_LIST _component_targets)
    set(arduino_component framework-arduinoespressif32)
else()
    set(arduino_component arduino)
endif()

set(requires
        ${arduino_component}
        bluepad32
        btstack)


idf_component_register(SRCS "${srcs}"
                INCLUDE_DIRS "include"
                REQUIRES ${requires})

 # Bluepad32 contains reference to setup() and loop().  If we add main
 # component to priv_requires then we create a large circular dependency
 # (bluepad32 -> main -> bluepad32) and can get linker errors, so
 # instead we add setup() and loop() to the undefined symbols list so the
 # linker will always include them.
 #
 # (As they are C++ symbol, we need to add the C++ mangled names.)
 set(ext_symbols "-u _Z6setupv -u _Z4loopv")

string(REPLACE ";" " " ext_symbols "${ext_symbols}")
target_link_libraries(${COMPONENT_LIB} INTERFACE ${ext_symbols})
