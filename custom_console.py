from pymavlink import mavutil
import argparse
import math


the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

received = the_connection.wait_heartbeat()

if(received):
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
else:
    print("Heartbeat was not received")


mode_id = the_connection.mode_mapping()
print(mode_id)

msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
print(msg)

altitude = the_connection.recv_match(type='ATTITUDE', blocking=True)
print(altitude)


modes = {'STABILIZE': 0, 
     'ACRO': 1, 
     'ALT_HOLD': 2, 
     'AUTO': 3, 
     'GUIDED': 4, 
     'LOITER': 5, 
     'RTL': 6, 
     'CIRCLE': 7, 
     'POSITION': 8, 
     'LAND': 9, 
     'OF_LOITER': 10, 
     'DRIFT': 11, 
     'SPORT': 13, 
     'FLIP': 14, 
     'AUTOTUNE': 15, 
     'POSHOLD': 16, 
     'BRAKE': 17, 
     'THROW': 18, 
     'AVOID_ADSB': 19, 
     'GUIDED_NOGPS': 20, 
     'SMART_RTL': 21, 
     'FLOWHOLD': 22, 
     'FOLLOW': 23, 
     'ZIGZAG': 24, 
     'SYSTEMID': 25, 
     'AUTOROTATE': 26, 
     'AUTO_RTL': 27}

custom_mode = 4
mode = 1
custom_submode = 0

arm = 0
force_arm = 0

pitch = 0
yaw = 0
longitude = 0
latitude = 0
altitude = 0

abort_altitude = 0
land_mode = 0

time_boot = 0
type_mask = 0
position_x = 0
position_y = 0
position_z = 0
velocity_x = 0
velocity_y = 0
velocity_z = 0
acceleration_x = 0
acceleration_y = 0
acceleration_z = 0
yaw_rate = 0



parser = argparse.ArgumentParser(prog='MAVLInk parser', description='Parses CMD arguments')

parser.add_argument('-m', '--mode', action="store",  type=int,help='Set the mode of your drone')
parser.add_argument('-cm', '--custom_mode', action="store", type=int, help='Set the custom mode of your drone')
parser.add_argument('-csm', '--custom_submode', action="store", type=int, help='Set the custom submode of your drone')
parser.add_argument('-a','--arm',action='store', type=int, help='Arm the component of your drone')
parser.add_argument('-fa', '--force_arm', action='store', type=int, help='Force arm the component without any safety and pre-flight checks')
parser.add_argument('-p', '--pitch', action='store', type=float, help='Set the pitch angle for takeoff')
parser.add_argument('-yaw', '--yaw', action='store', type=float, help='Set the yaw angle. NaN to use the current system yaw heading mode')
parser.add_argument('-la', '--latitude', action='store', type=float, help='Set the latitude')
parser.add_argument('-lo','--longitude', action='store', type=float, help='Set the longitude')
parser.add_argument('-alt','--altitude', action='store', type=float, help='Set the altitude')
parser.add_argument('-abalt', '--abort_altitude', action='store', type=float, help='Minimum target altitude if landing is aborted (0 = undefined/use system default)')
parser.add_argument('-lm', '--land_mode', action='store', type=float, help='Precision Land Mode:\n' \
                                                                    '0 - Normal (non-precision) landing' \
                                                                    '1 - Use precision landing if beacon detected when land command accepted, otherwise land normally' \
                                                                    '2 - Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found)')

parser.add_argument('-tb', '--time_boot', action='store', type=float, help='Time boot in milliseconds')
parser.add_argument('-tm', '--type_mask', action='store', type=int, help='Type Mask:' \
'Bitmask to indicate which fields should be ignored by the vehicle (see POSITION_TARGET_TYPEMASK enum)' \
'bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate' \
'When providing Pos, Vel and/or Accel all 3 axis must be provided. At least one of Pos, Vel and Accel must be provided (e.g. providing Yaw or YawRate alone is not supported)' \
'Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal' \
'Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)' \
'Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)' \
'Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)' \
'Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)' \
'Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)' \
'Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)'
)
parser.add_argument('-x', '--position_x', action='store', type=float, help='Position x in meters')
parser.add_argument('-y', '--position_y', action='store', type=float, help='Position y in meters')
parser.add_argument('-z', '--position_z', action='store', type=float, help='Position z in meters')
parser.add_argument('-vx', '--velocity_x', action='store', type=float, help='Velocity x in m/s')
parser.add_argument('-vy', '--velocity_y', action='store', type=float, help='Velocity y in m/s')
parser.add_argument('-vz', '--velocity_z', action='store', type=float, help='Velocity z in m/s')
parser.add_argument('-ax', '--acceleration_x', action='store', type=float, help='Acceleration x in m/s/s')
parser.add_argument('-ay', '--acceleration_y', action='store', type=float, help='Acceleration y in m/s/s')
parser.add_argument('-az', '--acceleration_z', action='store', type=float, help='Acceleration z in m/s/s')
parser.add_argument('-yawr', '--yaw_rate', action='store', type=float, help='yaw rate in rad/s')
args = parser.parse_args()
print(args)
def set_mode_parser():
    global custom_mode
    global mode
    global custom_submode
    if ((args.mode is not None) and (args.custom_mode is not None) and (custom_submode is not None)):
        mode = args.mode
        custom_mode = args.custom_mode
        custom_submode = args.custom_submode
    else:
        print("You have not inputted all the needed arguments. Try again.")
        exit()

    print(mode)
    while(True):
        if ((custom_mode > 27 or custom_mode < 0) or mode not in[1,2,4,8,16,32,64,128]):
            print("The values are out of scope, please try again.")
        else:
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mode, custom_mode, custom_submode, 0, 0, 0, 0)
            break

def arm_parser():
    global arm
    global force_arm
    if((args.arm is not None) and (args.force_arm is not None)):
        arm = args.arm
        force_arm = args.force_arm
    else:

        print("You have not inputted all the arguments, please try again")
    while(True):
        if((arm not in [0,1]) or force_arm not in [0, 21196]):
            print("The values are out of scope, please try again.")
        else:
            if(msg.custom_mode == 9):
                arm_manual()
                exit()
            else:
                the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, force_arm, 0, 0, 0, 0, 0)
                break
    

def takeoff_parser():
    global pitch, yaw, latitude, longitude, altitude
    if((pitch is not None) and (yaw is not None) and (latitude is not None) and (longitude is not None) and (altitude is not None)):
        pitch = float(args.pitch)
        yaw = float(args.yaw)
        latitude = float(args.latitude)
        longitude = float(args.longitude)
        altitude = float(args.altitude)
        if(custom_mode != 4):
            print("Impossible to take off in the current custom mode.")
        else:
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, pitch, yaw, 0, 0, latitude, longitude, altitude)
            
    else:
        print("You have not inputted all the needed arguments. Try again.")
        exit()


def land_parser():
    global abort_altitude, land_mode, yaw, longitude, altitude, latitude
    if((args.abort_altitude is not None) and (args.land_mode is not None) and (args.yaw is not None) and 
       (args.latitude is not None) and (args.longitude is not None) and (args.latitude is not None)):
        abort_altitude = args.abort_altitude
        land_mode = args.land_mode
        yaw = args.yaw
        longitude = args.longitude
        altitude = args.altitude
        latitude = args.latitude
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, abort_altitude, land_mode, 0,
                                              yaw, longitude, altitude, latitude)
    else:
        print("You have not inputted all the arguments, please try again")
        exit()

def move_parser():
    global type_mask, time_boot, position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, acceleration_x, acceleration_y, acceleration_z, yaw_rate, yaw
    if((args.time_boot is not None) and (args.type_mask is not None) and 
       (args.position_x is not None) and (args.position_y is not None) and (args.position_z is not None) and 
       (args.velocity_x is not None) and (args.velocity_y is not None) and (args.velocity_z is not None) and 
       (args.acceleration_x is not None) and (args.acceleration_y is not None) and (args.acceleration_z is not None) and 
       (args.yaw_rate is not None) and (args.yaw is not None)):
        time_boot = int(args.time_boot)
        type_mask = int(args.type_mask)
        position_x = int(args.position_x)
        position_y = int(args.position_y)
        position_z = int(args.position_z)
        velocity_x = int(args.velocity_x)
        velocity_y = int(args.velocity_y)
        velocity_z = int(args.velocity_z)
        acceleration_x = int(args.acceleration_x)
        acceleration_y = int(args.acceleration_y)
        acceleration_z = int(args.acceleration_z)
        yaw_rate = int(args.yaw_rate)
        yaw = int(args.yaw)
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(time_boot, the_connection.target_system, the_connection.target_component, 
                                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,  int(type_mask), position_x, position_y, position_z, 
                                                                                          velocity_x,velocity_y,velocity_z,
                                                                                          acceleration_x,acceleration_y,acceleration_z,
                                                                                          yaw,yaw_rate))
    else:
        print("You have not inputted enough arguments, please try again")
        exit()

        


def set_mode():
    global custom_mode
    print(modes)
    while(True):
        custom_mode = int(input("Choose your mode:\n "))
        if (custom_mode > 27 or custom_mode < 0):
            print("the values are out of scope, please try again.")
        else:
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, custom_mode, 0, 0, 0, 0, 0)
            break


def arm_manual():
        if(msg.custom_mode == 9):
             print("Impossible to arm in LAND mode. Do you want to change the mode to any of other custom modes? y/n")
             user_answer = str(input())
             match user_answer: 
                case "y":
                        set_mode()
                        for i, k in enumerate(modes):
                            if i == custom_mode:
                                print(f"Custom mode {k} was set succesfully. Run the program again in order to use applied changes.")

                    
                case "n":
                     exit()
                     
        else:
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


            
def takeoff():
    global custom_mode
    if(custom_mode != 4):
        print("Impossible to take off in the current custom mode.")
    else:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
         
def land():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 10, 0, 0, 0, 0, 0, 0, 10)



def move():
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component, 
                                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,  int(0b110111111000), 20, 0, -10, 0,0,0,0,0,0,0,0))

def main():
    #set_mode_parser()
    #set_mode()
    #arm_manual() 
    #arm_parser()
    #land_parser()
    #takeoff()
    #takeoff_parser()
    #move()
    move_parser()
    #land()
main()
