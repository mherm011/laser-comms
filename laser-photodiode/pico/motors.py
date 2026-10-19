# motors.py - hook for the gimbal team (TX Pico only).
#
# The Pi 5 sends lines like "MOT <anything>" over USB; main.py passes the
# text after "MOT" to handle(). service() is called every loop pass (~1 ms),
# so it can run a control loop, ramp step rates, etc.
#
# The laser is driven by the hardware UART, so nothing done here can corrupt
# laser bits - the worst a slow handler can do is delay the start of the next
# frame. Keep handlers short anyway (no long sleeps). Use machine.PWM or rp2
# PIO for servo pulses / step pulses so their timing is in hardware too.
#
# Pins already taken: GP16 (laser), GP1 (UART0 RX, reserved), and in BOTH mode
# GP4/GP5/GP26 (receiver). Anything else is free.


def handle(args):
    # Example protocol the gimbal code could use: "MOT <pan_deg> <tilt_deg>"
    return "ERR motors not implemented yet (edit motors.py): " + args


def service():
    pass
