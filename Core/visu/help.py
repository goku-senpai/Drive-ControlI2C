import tkinter as tk
from tkinter import ttk

# Define the help text for each motor type
dc_help_text = "To calculate the PID gains for a DC motor, you need to measure the motor's speed and current while it's running under load. Then, you can use the following formula:\n\nL = Kt / R\n\nWhere L is the PID gain, Kt is the torque constant, and R is the armature resistance."

stepper_help_text = "To calculate the PID gains for a stepper motor, you need to measure the motor's torque and speed while it's running under load. Then, you can use the following formula:\n\nL = (Kt / R) * (60 / S)\n\nWhere L is the PID gain, Kt is the torque constant, R is the winding resistance, and S is the stepping rate in steps per second."

servo_help_text = "To calculate the PID gains for a servo motor, you need to measure the motor's velocity and torque while it's running under load. Then, you can use the following formula:\n\nL = Kt / Kb\n\nWhere L is the PID gain, Kt is the torque constant, and Kb is the back-emf constant."

zn_help_text = "The (proportional) gain, K p is increased (from zero) until it reaches the ultimate gain K u, at which the output of the control loop has stable and consistent oscillations. K u K_{u} and the oscillation period T u T_{u} are then used to set the P, I, and D gains depending on the type of controller used and behaviour desired. Classic PID: K_p:0.6xK_u; Ki:1.2x K_u/T_u; K_p:0.075xK_uxT_u"

# Define a function to show the help text based on the user's selection
def show_help_text():
    selection = motor_type.get()
    if selection == "DC":
        help_label.configure(text=dc_help_text)
    elif selection == "Stepper":
        help_label.configure(text=stepper_help_text)
    elif selection == "Servo":
        help_label.configure(text=servo_help_text)
    elif selection == "Ziegler–Nichols":
        help_label.configure(text=zn_help_text)
    else:
        help_label.configure(text="Please select a motor type.")

# Create the main window
root = tk.Tk()
root.title("Motor PID Gains Help")

# Create the drop-down menu
motor_type = ttk.Combobox(root, values=["DC", "Stepper", "Servo", "Ziegler–Nichols"])
motor_type.pack()

# Create the help label
help_label = tk.Label(root, wraplength=400, justify="left")
help_label.pack()

# Create the "Show Help" button
help_button = tk.Button(root, text="Show Help", command=show_help_text)
help_button.pack()

# Add a Windows drop-down menu
menu = tk.Menu(root)
root.config(menu=menu)
root.configure(background="#474757")  # set background to #474757
file = tk.Menu(menu)
file.add_command(label='Exit', command=root.quit)
menu.add_cascade(label='File', menu=file)

# Add a "Help" dropdown menu
help_menu = tk.Menu(menu)
help_menu.add_command(label="Motor PID Gains", command=lambda: help_label.configure(text="This tool helps calculate the PID gains for various types of motors. Select a motor type from the drop-down menu and click the 'Show Help' button to see the calculation formula."))
menu.add_cascade(label="Help", menu=help_menu)

root.mainloop()
