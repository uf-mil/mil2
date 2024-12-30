from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the 'simulated' argument
    declare_simulated_arg = DeclareLaunchArgument(
        'simulated',
        default_value='False',
        description='Set to True if running in simulation'
    )

    # Set 'is_simulation' parameter based on the 'simulated' argument
    set_is_simulation_param = SetLaunchConfiguration(
        name='is_simulation',
        value=LaunchConfiguration('simulated')
    )

    # Define the node
    usb_to_can_driver_node = Node(
        package='mil_usb_to_can',
        executable='sub9_driver',
        name='usb_to_can_driver',
        parameters=[{
            'is_simulation': LaunchConfiguration('simulated'),
            'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A900KV22-if00-port0',
            'baudrate': 115200,
            'can_id': 0,
            'device_handles': {
                "3": "mil_usb_to_can.ExampleEchoDeviceHandle"
            },
            'simulated_devices': {
                "3": "mil_usb_to_can.ExampleSimulatedEchoDevice"
            }
        }]
    )

    return LaunchDescription([
        declare_simulated_arg,
        set_is_simulation_param,
        usb_to_can_driver_node
    ])
