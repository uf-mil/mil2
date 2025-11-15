// Copyright 2025 University of Florida Machine Intelligence Laboratory
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <cstdint>

namespace navigator_ball_launcher
{

/**
 * @brief Packet to set the spin state of the ball launcher motor.
 *
 * class_id: 0x42, subclass_id: 0x01
 * Payload format: single bool encoded as byte
 */
struct SetSpinPacket
{
    static constexpr uint8_t CLASS_ID = 0x42;
    static constexpr uint8_t SUBCLASS_ID = 0x01;

    bool spin;

    explicit SetSpinPacket(bool s = false) : spin(s)
    {
    }
};

/**
 * @brief Packet to release/drop a ball from the launcher.
 *
 * class_id: 0x42, subclass_id: 0x02
 * Payload format: empty (no data)
 */
struct ReleaseBallPacket
{
    static constexpr uint8_t CLASS_ID = 0x42;
    static constexpr uint8_t SUBCLASS_ID = 0x02;

    ReleaseBallPacket() = default;
};

}  // namespace navigator_ball_launcher
