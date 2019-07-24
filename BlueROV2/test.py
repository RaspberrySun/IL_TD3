# def save_data(filename, depth_data, roll_data, pitch_data, pwm):
#     with open(filename, 'a') as f:
#         f.writelines(str(depth_data)[:5] + '  ' + str(roll_data)[:7] + '  ' + str(pitch_data)[:7] + '  ' + str(pwm[0])
#                      + '  ' + str(pwm[1])+'  '+str(pwm[2])+'  '+str(pwm[3])+'\n')
#
# filename = 'data.txt'
# save_data(filename, 'Depth', 'Roll', 'Pitch', ['PWM1', 'PWM2', 'PWM3', 'PWM4'])
#
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Get some information !
while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'RC_CHANNELS_RAW':

        print(msg)
