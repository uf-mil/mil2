#pragma once

#include <string>

class NoopSerial
{
  public:
    std::string port = "noop-serial";

    NoopSerial();
    ~NoopSerial();

    virtual void open();
    virtual int in_waiting() const;
    virtual int out_waiting() const;
    virtual void close();
    virtual std::string read(int length);
    virtual int write(std::string const& data);
    virtual void flush();
    virtual void flushInput();
    virtual void flushOutput();
    virtual void reset_input_buffer();
    virtual void reset_output_buffer();
    virtual void send_break();
};
