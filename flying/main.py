import math

from djitellopy import tello
from time import sleep, perf_counter
from rc_controls import controls

dt = 0.1

d = tello.Tello()
d.connect()

x_pred = 0
y_pred = 0
z_pred = 0
yaw_pred = 0

x_with_speed = 0
y_with_speed = 0
z_with_speed = 0

x_with_acceleration = 0
y_with_acceleration = 0
z_with_acceleration = 0

d.takeoff()
for control in controls:
    t = perf_counter()
    # moving to get real values (except for the offset in the z direction while takeoff)
    d.send_rc_control(left_right_velocity=control[0],
                      forward_backward_velocity=control[1],
                      up_down_velocity=control[2],
                      yaw_velocity=control[3])

    pitch = d.get_pitch()
    roll = d.get_roll()
    yaw = d.get_yaw()

    x_pred += control[0] * dt
    y_pred += control[1] * dt
    z_pred += control[2] * dt
    yaw_pred += control[3] * dt

    x_with_speed += + d.get_speed_x() * dt * math.cos(yaw) + d.get_speed_y() * dt * math.sin(yaw)
    y_with_speed += - d.get_speed_x() * dt * math.sin(yaw) + d.get_speed_y() * dt * math.cos(yaw)
    z_with_speed += d.get_speed_z() * dt

    x_with_acceleration += d.get_acceleration_x()
    y_with_acceleration += d.get_acceleration_y()
    z_with_acceleration += d.get_acceleration_z()

    height = d.get_height()
    barometer_height = d.get_barometer()

    print(f'{x_pred}, {x_with_speed}')
    print(f'{y_pred}, {y_with_speed}')
    print(f'{z_pred}, {z_with_speed}')
    print(f'{yaw_pred}, {yaw}')

    # print(f"Error in yaw: {0 if y_pred == 0.0 else (yaw_pred-yaw)/yaw_pred}")
    # print(f"Error using speed: {(x_pred - x_with_speed) / x_pred}, {(y_pred - y_with_speed) / y_pred}, {(z_pred - z_with_speed) / z_pred}")
    # print(f"Error using acceleration: {(x_pred - x_with_acceleration) / x_pred}, {(y_pred - y_with_acceleration) / y_with_acceleration}, {(z_pred - z_with_acceleration) / z_pred}")
    # print(f"Height: {height}, Error in Height: {(z_pred - height) / z_pred}")
    # print(f"Barometer Height: {barometer_height}, Error in Height: {(z_pred - barometer_height) / z_pred}")
    print('\n\n\n\n')

    sleep(max(dt - (perf_counter() - t), 0))


d.land()