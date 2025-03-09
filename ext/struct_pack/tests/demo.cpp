#include "struct_pack.hpp"

#include <iostream>

auto main() -> int {
    auto packed = struct_pack::pack(PY_STRING(">h"), 0x03ff); // 1023
    std::cout << (static_cast<unsigned char>(packed[0]) == 0x03) << '\n';
    std::cout << (static_cast<unsigned char>(packed[1]) == 0xff) << '\n';
}
