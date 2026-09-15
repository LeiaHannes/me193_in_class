import time

import legoeducation as le

# connect to the Double Motor
card_color = le.LEGO_COLOR_AZURE
card_serial = '0997'

doublemotor = le.DoubleMotor()
doublemotor.connect(card_color=card_color, card_serial=card_serial)

if not doublemotor.connected:
    print('Error connecting to Double Motor.')
    exit(1)


doublemotor.disconnect()