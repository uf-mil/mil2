#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <map>
#include <string>
#include <vector>

namespace navigator_kill_board
{

/**
 * @brief No-operation serial interface for testing purposes
 *
 * Inherits from a basic serial interface, doing nothing for each function.
 * Allows subclasses to implement custom behavior for simulating serial devices.
 */
class NoopSerial
{
  public:
    std::string port = "noop-serial";

    NoopSerial();
    virtual ~NoopSerial();

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

/**
 * @brief Simulates a serial device with buffered I/O
 *
 * Stores a buffer to be read like a normal OS serial device.
 * Intended to be extended by other classes, which should override the write
 * function to receive writes to the simulated device.
 */
class SimulatedSerial : public NoopSerial
{
  protected:
    std::string buffer_;  //!< Internal buffer for simulated serial data

  public:
    SimulatedSerial();
    virtual ~SimulatedSerial() = default;

    int in_waiting() const override;
    void reset_input_buffer() override;
    std::string read(int length) override;
};

/**
 * @brief Simulates Navigator's kill board over serial
 *
 * Pretends to be Navigator's kill board, responding according to the protocol
 * to requests and sending current state periodically.
 */
class SimulatedKillBoard : public SimulatedSerial
{
  private:
    ros::Time last_ping_;                              //!< Timestamp of last ping received
    bool has_last_ping_;                               //!< Whether we have received any ping
    std::map<std::string, bool> memory_;               //!< Kill state memory
    bool killed_;                                      //!< Overall kill status
    std::string light_;                                //!< Current light status
    ros::Timer timer_;                                 //!< Timer for periodic checks
    std::vector<ros::ServiceServer> service_servers_;  //!< ROS service servers

    /**
     * @brief Service callback for button presses
     *
     * @param button The name of the button
     * @param req Service request containing button state
     * @param res Service response
     * @return true if successful
     */
    bool set_button_callback(std::string const& button, std_srvs::SetBool::Request& req,
                             std_srvs::SetBool::Response& res);

    /**
     * @brief Timer callback for periodic operations
     *
     * @param event Timer event (unused)
     */
    void timer_callback(ros::TimerEvent const& event);

    /**
     * @brief Check for heartbeat timeout and update kill state
     */
    void check_timeout();

    /**
     * @brief Set kill state for a specific source
     *
     * @param name Name of the kill source
     * @param on Whether the kill is active
     * @param update Whether to update the buffer with responses
     */
    void set_kill(std::string const& name, bool on, bool update = true);

    /**
     * @brief Set light status
     *
     * @param status Light status string ("OFF", "YELLOW", "GREEN")
     */
    void set_light(std::string const& status);

    /**
     * @brief Get status response for a specific request byte
     *
     * @param byte The request byte to respond to
     */
    void get_status(char byte);

    /**
     * @brief Handle synchronous protocol requests
     *
     * @param data The request byte received
     */
    void handle_sync(char data);

    /**
     * @brief Helper function to get response character
     *
     * @param boolean The boolean value to convert
     * @return Response character (RESPONSE_TRUE or RESPONSE_FALSE)
     */
    char get_response(bool boolean) const;

  public:
    std::string port = "simulated-kill-board";

    /**
     * @brief Construct a new Simulated Kill Board
     */
    SimulatedKillBoard();

    /**
     * @brief Destructor
     */
    virtual ~SimulatedKillBoard() = default;

    /**
     * @brief Write data to the simulated kill board
     *
     * @param data Data to write to the board
     * @return Number of bytes written
     */
    int write(std::string const& data) override;
};

}  // namespace navigator_kill_board
