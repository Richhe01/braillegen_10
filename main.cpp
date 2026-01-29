#include <emscripten.h> // ignore error, load emscripten before liblouis tries to

#include <iostream>
#include "liblouis/liblouis.h"

int main() {
    // 1. Define the version string pointer
    const char *version;

    // 2. Ask liblouis for its version
    version = lou_version();

    // 3. Print the result
    std::cout << "Build successful!" << std::endl;
    std::cout << "Liblouis Version: " << version << std::endl;

    return 0;
}