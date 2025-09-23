#include "simulated_kill_board.h"

#include <iostream>
#include <string>
#include <vector>

#include "constants.h"

using namespace std;

/*
===========================================================================================
                            NoopSerial
===========================================================================================
This is a no-operation serial class that simulates a serial port for testing purposes.
All methods are implemented to do nothing or return default values.
*/

// Constructor
NoopSerial::NoopSerial()
{
    // Do nothing
}

// Destructor
NoopSerial::~NoopSerial()
{
    // Do nothing
}

void NoopSerial::open()
{
    // Do nothing
}

int NoopSerial::in_waiting() const
{
    return 0;
}

int NoopSerial::out_waiting() const
{
    return 0;
}

void NoopSerial::close()
{
    // Do nothing
}

string NoopSerial::read(int length)
{
    return "";
}

int NoopSerial::write(string const& data)
{
    return static_cast<int>(data.length());
}

void NoopSerial::flush()
{
    // Do nothing
}

void NoopSerial::flushInput()
{
    // Do nothing
}

void NoopSerial::flushOutput()
{
    // Do nothing
}

void NoopSerial::reset_input_buffer()
{
    // Do nothing
}

void NoopSerial::reset_output_buffer()
{
    // Do nothing
}

void NoopSerial::send_break()
{
    // Do nothing
}
