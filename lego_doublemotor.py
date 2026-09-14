import time

import legoeducation as le

# Update these to match your motor's Connection Card
card_color = le.LEGO_COLOR_AZURE
card_serial = '0997'

# Connect to the Double Motor
doublemotor = le.DoubleMotor()
doublemotor.connect(card_color=card_color, card_serial=card_serial)

# Check connection
if not doublemotor.connected:
    print('Error connecting to Double Motor.')
    exit(1)

# Optional: reset yaw to 0 at startup so readings are relative to this orientation
doublemotor.imu_reset_yaw_axis(0)

# Read and print yaw for five seconds
for i in range(50):
    yaw = doublemotor.imu_device.yaw
    doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_COUNTERCLOCKWISE, motor=le.MOTOR_LEFT, speed=100)
    print(f"Yaw: {yaw}")
    time.sleep(0.1)

# Disconnect
doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_COUNTERCLOCKWISE, motor=le.MOTOR_LEFT, speed=0)
doublemotor.disconnect()
exit(0)