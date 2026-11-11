# main needs no REQUIRES: ESP-IDF links every other component into it.
idf_component_register(SRCS "app_main.c" "main.cpp" "gamepad.cpp"
                       INCLUDE_DIRS "." "../include")
