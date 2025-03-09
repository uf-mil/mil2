// This file will test the implementation against Python's output
#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>

#include "struct_pack.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#define CATCH_CONFIG_ENABLE_TUPLE_STRINGMAKER
#include <catch2/catch.hpp>

#if 0
template <typename T>
std::string escapeString(const T &val) {
    std::string out = "\"";
    for (auto c : val) {
        if (isgraph(c)) {
            out.push_back(c);
        } else if (isprint(c)) {
            switch (c) {
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
            }
        } else {
            std::stringstream s;
            s << std::setfill('0') << std::setw(2) << static_cast<int>(c);
            out += "\\" + s.str();
        }
    }

    out += '"';
    return out;
}

template <typename T>
std::string convertToString(const T &val) {
    if constexpr (std::is_same_v<T, char>) {
        return "chr(" + std::to_string(val) + ")";
    } else if constexpr (std::is_integral_v<T>) {
        return std::to_string(val);
    } else if constexpr (std::is_convertible_v<T, std::string>) {
        return escapeString(val);
    } else if constexpr (std::is_floating_point_v<T>) {
        std::stringstream ss;
        ss << std::scientific << std::setprecision(16) << val;
        return ss.str();
    } else {
        return Catch::StringMaker<T>::convert(val);
    }
}

template <typename Fmt, typename... Args>
std::string
buildPythonScript(Fmt, const std::string &outputPath, const Args &...toPack) {
    std::string pythonScript = R"py(
# Workaround for Catch::StringMaker<bool>
true = True
false = False

import struct
bin = struct.pack()py" + convertToString(Fmt::value());

    std::string args[] = {convertToString(toPack)...};
    for (const auto &arg : args) {
        pythonScript += ", ";
        pythonScript += arg;
    }

    pythonScript += R"py()
with open(r')py" + outputPath
                    + R"py(', 'wb') as f:
    f.write(bin)
    )py";

    return pythonScript;
}

template <typename Fmt, typename... Args>
auto runPythonPack(Fmt, const Args &...toPack) {
    // Create the python script
    std::string packedBinaryFilePath = std::tmpnam(nullptr);
    std::string pythonScript
        = buildPythonScript(Fmt{}, packedBinaryFilePath, toPack...);
    std::cout << pythonScript << std::endl;

    // Write the python script
    std::string pythonScriptPath = std::tmpnam(nullptr);
    auto pythonScriptFile = std::ofstream(pythonScriptPath, std::ios::binary);
    pythonScriptFile.write(pythonScript.data(), pythonScript.size());
    pythonScriptFile.close(); // Flush to disk

    // Run the python script
    std::string scriptRunCommand = "python " + pythonScriptPath;
    auto        retVal = system(scriptRunCommand.c_str());
    if (retVal != 0) {
        FAIL("Python run failed with error code " << retVal);
    }

    // Read the python script output
    std::ifstream inputFile(packedBinaryFilePath, std::ios::binary);
    inputFile = std::ifstream(packedBinaryFilePath, std::ios::binary);
    std::vector<char> outputBuffer((std::istreambuf_iterator<char>(inputFile)),
                                   (std::istreambuf_iterator<char>()));

    inputFile.close();

    // Remove script+output files
    remove(pythonScriptPath.c_str());
    remove(packedBinaryFilePath.c_str());

    return outputBuffer;
}

template <typename Fmt, typename... Args>
void testPackAgainstPython(Fmt, const Args &...toPack) {
    auto packed = struct_pack::pack(Fmt{}, toPack...);
    auto pythonPacked = runPythonPack(Fmt{}, toPack...);

    CAPTURE(packed);
    CAPTURE(pythonPacked);

    REQUIRE(packed.size() == pythonPacked.size());
    REQUIRE(std::equal(packed.begin(), packed.end(), pythonPacked.begin()));

    auto unpacked = struct_pack::unpack(Fmt{}, pythonPacked);
    REQUIRE(unpacked == std::make_tuple(toPack...));
}

#define REULAR_STRING(str) PY_STRING(str)
#define NATIVE_STRING(str) PY_STRING("@" str)
#define NATIVE_SIZE_STRING(str) PY_STRING("=" str)
#define BIG_ENDIAN_STRING(str) PY_STRING(">" str)
#define NETWORK_ENDIAN_STRING(str) PY_STRING("!" str)
#define LITTLE_ENDIAN_STRING(str) PY_STRING("<" str)

#define TEST_PACK(str, ...)                                                    \
    testPackAgainstPython(REULAR_STRING(str), __VA_ARGS__);                    \
    testPackAgainstPython(NATIVE_STRING(str), __VA_ARGS__);                    \
    testPackAgainstPython(NATIVE_SIZE_STRING(str), __VA_ARGS__);               \
    testPackAgainstPython(BIG_ENDIAN_STRING(str), __VA_ARGS__);                \
    testPackAgainstPython(NETWORK_ENDIAN_STRING(str), __VA_ARGS__);            \
    testPackAgainstPython(LITTLE_ENDIAN_STRING(str), __VA_ARGS__);

TEST_CASE("integers - single items", "[cpstruct_pack::binary_compat]") {
    TEST_PACK("?", false);
    TEST_PACK("?", true);
    TEST_PACK("c", 'a');

    // Signed - positive
    TEST_PACK("b", 126);
    TEST_PACK("h", 126);
    TEST_PACK("i", 126);
    TEST_PACK("l", 126);
    TEST_PACK("q", 126);

    // Signed - negative
    TEST_PACK("b", -126);
    TEST_PACK("h", -126);
    TEST_PACK("i", -126);
    TEST_PACK("l", -126);
    TEST_PACK("q", -126);

    // Unsigned
    TEST_PACK("B", std::numeric_limits<unsigned char>::max());
    TEST_PACK("H", std::numeric_limits<unsigned short>::max());
    TEST_PACK("I", std::numeric_limits<unsigned int>::max());
    TEST_PACK("L",
              std::numeric_limits<unsigned int>::max()); // L is unsigned int
                                                         // with standard sizing
    TEST_PACK("Q", std::numeric_limits<unsigned long long>::max());
}

TEST_CASE("strings - single items", "[cpstruct_pack::binary_compat]") {
    std::string s = "This is a test";
    s.resize(50);
    TEST_PACK("50s", s);

    s.resize(500);
    TEST_PACK("500s", s);

    s.resize(5);
    TEST_PACK("5s", s);
}

TEST_CASE("floats - single items", "[cpstruct_pack::binary_compat]") {
    TEST_PACK("f", 0);
    TEST_PACK("d", 0);

    TEST_PACK("f", std::numeric_limits<float>::max());
    TEST_PACK("d", std::numeric_limits<double>::max());

    TEST_PACK("f", std::numeric_limits<float>::min());
    TEST_PACK("d", std::numeric_limits<double>::min());
}

TEST_CASE("pack complex formats", "[cpstruct_pack::binary_compat]") {
    TEST_PACK("2cf3s2H?", 'x', 'y', 0.5, "zwt", 0x1234, 0x5678, false);
    TEST_PACK("L2cd5si?", 0x1234UL, 'l', 'o', 3.14159, "lolzs", -500, true);
    TEST_PACK("bhi?lfq",
              127,
              32767,
              2147483647,
              true,
              2147483647,
              123456,
              9223372036854775807);
    TEST_PACK("bhil?fq",
              -127,
              -32767,
              -2147483647,
              -2147483647,
              false,
              -500,
              -9223372036854775807);
}
#endif
