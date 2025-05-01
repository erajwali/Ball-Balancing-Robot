import sensor, image, time
from pyb import UART

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Initialize UART
uart = UART(3, 115200)

# Threshold for orange color detection
orange_threshold = (63, 83, -6, 127, 58, 127)

# Center coordinates of platform (Will be different for your application)
target_x = 125
target_y = 71

# PID constants
Kp = 1.5 # Proportional gain
Kd = 0.4 # Derivative gain
Ki = 0.02 # Integral gain
scale_factor = 10 # Scale factor for control signal

# Previous error for derivative term
prev_error_x = 0
prev_error_y = 0

# Time differential for Integral term
prev_time = time.ticks_ms()

# Cumulative integral term
integral_sum_x = 0
integral_sum_y = 0


while True:
    img = sensor.snapshot()
    blobs = img.find_blobs([orange_threshold], pixels_threshold=100, area_threshold=100, merge=True)

    if blobs:
        # Find the largest blob
        largest_blob = max(blobs, key=lambda b: b.pixels())

        # Draw rectangle around largest blob
        img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))
        img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=(255, 0, 0))


        # Determine ball's position error
        ball_x = largest_blob.cx()
        ball_y = largest_blob.cy()
        error_x = target_x - ball_x
        error_y = target_y - ball_y

        # Calculate time differential
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, prev_time)/1000 # Convert to seconds
        prev_time = current_time

        # Calculate PID control signals for x (Integral term separate)
        proportional_x = Kp*error_x
        derivative_x = (Kd*(error_x-prev_error_x))/dt # Be sure to divide by differential
        prev_error_x = error_x

        # Calculate PID control signals for y (Integral term separate)
        proportional_y = Kp*error_y
        derivative_y = (Kd*(error_y-prev_error_y))/dt
        prev_error_y = error_y

        # Integral term for x
        integral_x = Ki * error_x * dt
        integral_sum_x += integral_x # Update cumulative integral term for numerical integration
        # Integral term for y
        integral_y = Ki * error_y * dt
        integral_sum_y += integral_y # Update cumulative integral term numerical integration


        # Control signal for x
        control_signal_x = (proportional_x + integral_sum_x + derivative_x)*scale_factor
        #Control signal for y
        control_signal_y = (proportional_y + integral_sum_y + derivative_y)*scale_factor

        # Send control signals over UART
        uart.write(f"{control_signal_x},{control_signal_y}\n")








