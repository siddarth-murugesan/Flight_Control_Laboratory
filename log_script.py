#references
#https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/
#https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
#Checkout logging_variables_info.txt file in the repo for the available firmware variables to log

# headers
import logging
import time
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Set the default flight height of the drone (in meters)
DEFAULT_HEIGHT = 0.6

# Flowdeck event declaration
deck_attached_event = Event()
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Function for drone movement
def check_test_f_r_move(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        print('Start')
        mc.forward(2)
        time.sleep(1)
        mc.stop()

# Callback function for parameter logging
def log_pos_callback(timestamp, data, logconf):
    print(data)

# Deck verification function for parameter logging
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

# Main function
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Flow deck verification
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        time.sleep(1)

        # Logging variables details (only 6 system variables can be logged at a time)
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.z', 'float')
        #logconf.add_variable('kalman.stateF', 'float')
        logconf.add_variable('kalman.tofsensorpreF', 'float')
        logconf.add_variable('kalman.tofsensormeaF', 'float')
        logconf.add_variable('kalman.toferrorF', 'float')
        logconf.add_variable('kalman.tofthresF', 'float')
        #logconf.add_variable('kalman.stateR', 'float')
        #logconf.add_variable('kalman.tofsensorpreR', 'float')
        #logconf.add_variable('kalman.tofsensormeaR', 'float')
        #logconf.add_variable('kalman.toferrorR', 'float')
        #logconf.add_variable('kalman.tofthresR', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # If the deck is not attached, then program exit
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Start logging and drone movement
        logconf.start()
        check_test_f_r_move(scf)
        # End logging
        logconf.stop()

        
