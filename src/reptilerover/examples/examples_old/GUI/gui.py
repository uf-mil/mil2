import tkinter as tk
import customtkinter as ctk
import subprocess
import lgpio

h = lgpio.gpiochip_open(0)

# Define the initial key bindings (default to WASD)
key_bindings = {
    'forward': 'w',
    'backward': 's',
    'left': 'a',
    'right': 'd',
    'stop': 'e'
}

def send_command(command):
    print(command)
    left_motors.value = 0.2
    

# Function to apply button shading on press
def on_button_press(button):
    button.configure(fg_color="gray")  # Change color when pressed

# Function to remove shading and send command when key/button is released
def on_button_release(button, command):
    button.configure(fg_color="blue")  # Revert to original color when released
    send_command(command)

# Function to update key bindings to arrow keys
def set_arrow_keys():
    global key_bindings
    key_bindings = {
        'forward': '<Up>',
        'backward': '<Down>',
        'left': '<Left>',
        'right': '<Right>',
        'stop': '<space>'
    }
    update_button_texts()
    bind_keys()

# Function to update key bindings to WASD keys
def set_wasd_keys():
    global key_bindings
    key_bindings = {
        'forward': 'w',
        'backward': 's',
        'left': 'a',
        'right': 'd',
        'stop': 'e'
    }
    update_button_texts()
    bind_keys()

# Function to update button labels based on the key bindings
def update_button_texts():
    forward_button.configure(text=f"Forward ({key_bindings['forward'].strip('<>')})")
    backward_button.configure(text=f"Backward ({key_bindings['backward'].strip('<>')})")
    left_button.configure(text=f"Left ({key_bindings['left'].strip('<>')})")
    right_button.configure(text=f"Right ({key_bindings['right'].strip('<>')})")
    stop_button.configure(text=f"Stop ({key_bindings['stop'].strip('<>')})")

# Function to bind the selected keys to the buttons
def bind_keys():
    # Unbind previous key bindings
    root.unbind_all('<KeyPress>')
    root.unbind_all('<KeyRelease>')

    # Bind keys to corresponding button actions
    root.bind(f"<KeyPress-{key_bindings['forward'].strip('<>')}>", lambda e: on_button_press(forward_button))
    root.bind(f"<KeyRelease-{key_bindings['forward'].strip('<>')}>", lambda e: on_button_release(forward_button, key_bindings['forward']))

    root.bind(f"<KeyPress-{key_bindings['backward'].strip('<>')}>", lambda e: on_button_press(backward_button))
    root.bind(f"<KeyRelease-{key_bindings['backward'].strip('<>')}>", lambda e: on_button_release(backward_button, key_bindings['backward']))

    root.bind(f"<KeyPress-{key_bindings['left'].strip('<>')}>", lambda e: on_button_press(left_button))
    root.bind(f"<KeyRelease-{key_bindings['left'].strip('<>')}>", lambda e: on_button_release(left_button, key_bindings['left']))

    root.bind(f"<KeyPress-{key_bindings['right'].strip('<>')}>", lambda e: on_button_press(right_button))
    root.bind(f"<KeyRelease-{key_bindings['right'].strip('<>')}>", lambda e: on_button_release(right_button, key_bindings['right']))

    root.bind(f"<KeyPress-{key_bindings['stop'].strip('<>')}>", lambda e: on_button_press(stop_button))
    root.bind(f"<KeyRelease-{key_bindings['stop'].strip('<>')}>", lambda e: on_button_release(stop_button, key_bindings['stop']))

# Set the theme and appearance mode (dark/light)
ctk.set_appearance_mode("dark")  
ctk.set_default_color_theme("blue")  

# Create the main window
root = ctk.CTk()
root.title("Rover Control Panel")
root.geometry("400x300")

# Add a settings menu using tkinter's Menu
menu_bar = tk.Menu(root)
settings_menu = tk.Menu(menu_bar, tearoff=0)

# Add options to the settings menu to switch key bindings
settings_menu.add_command(label="Use Arrow Keys", command=set_arrow_keys)
settings_menu.add_command(label="Use WASD Keys", command=set_wasd_keys)

# Add the settings menu to the menu bar
menu_bar.add_cascade(label="Settings", menu=settings_menu)

# Set the menu bar in the window
root.config(menu=menu_bar)

# Create buttons and bind press/release events
forward_button = ctk.CTkButton(root, text="Forward (w)", width=100, height=40)
backward_button = ctk.CTkButton(root, text="Backward (s)", width=100, height=40)
left_button = ctk.CTkButton(root, text="Left (a)", width=100, height=40)
right_button = ctk.CTkButton(root, text="Right (d)", width=100, height=40)
stop_button = ctk.CTkButton(root, text="Stop (e)", width=100, height=40)

# Position the buttons on the GUI
forward_button.grid(row=0, column=1, pady=20)
backward_button.grid(row=2, column=1, pady=20)
left_button.grid(row=1, column=0, padx=20)
right_button.grid(row=1, column=2, padx=20)
stop_button.grid(row=1, column=1, pady=20)

# Bind the default keys (WASD) when the GUI starts
bind_keys()

# Start the GUI event loop
root.mainloop()
