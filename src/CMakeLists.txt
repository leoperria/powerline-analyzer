# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

## Component sources
## NOTE: Do NOT use GLOB here. ESP-IDF and PlatformIO both warn against it
## because CMake cannot detect new/removed files without a reconfigure, and
## a recursive glob can pick up stray files from IDE build folders
## (e.g. src/cmake-build-debug/.../CompilerIdC/CMakeCCompilerId.c).
set(app_sources
        "main.c"
)

idf_component_register(
        SRCS ${app_sources}
        INCLUDE_DIRS "."
        REQUIRES esp_adc esp_driver_usb_serial_jtag
)
