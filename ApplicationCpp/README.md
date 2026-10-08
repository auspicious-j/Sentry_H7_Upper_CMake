# ApplicationCpp

This directory contains user-owned C++17 application and plugin framework code.

CubeMX-generated C files remain under `Core/`, `Drivers/`, `USB_DEVICE/`, and
`Middlewares/`. The C++ layer enters through the small `extern "C"` bridge in
`app_cpp_entry.h`; it does not replace `main.c` or `freertos.c`.
