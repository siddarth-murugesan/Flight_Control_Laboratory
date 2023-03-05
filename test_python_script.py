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
DEFAULT_HEIGHT = 1.1

# Flowdeck and Multiranger deck event declaration
flow_deck_attached_event = Event()
mr_deck_attached_event = Event()
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Callback function for paramter setting
def param_stab_est_callback(name, value):
    print('The crazyflie detection factor ' + name + ' set at number: ' + value)

# Paramter setting function (setting value for detection factor variable based on set height)   
def simple_set_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr + '.' + namestr
    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    if DEFAULT_HEIGHT == 0.6:
        cf.param.set_value(full_name, 3.25)
    elif DEFAULT_HEIGHT == 1.1:
        cf.param.set_value(full_name, 3.5)
    elif DEFAULT_HEIGHT == 1.5:
        cf.param.set_value(full_name, 3.75)
    elif DEFAULT_HEIGHT == 2.0:
        cf.param.set_value(full_name, 5.5)
    else:
        cf.param.set_value(full_name, 10)
    time.sleep(1)

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
        flow_deck_attached_event.set()
        print('FlowDeck is attached!')
    else:
        print('FlowDeck is NOT attached!')

# Deck verification function for parameter logging
def param_multiranger(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        mr_deck_attached_event.set()
        print('Multiranger Deck is attached!')
    else:
        print('Multiranger Deck is NOT attached!')

# Main function
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Parameter variable details
    group = 'kalman'
    name = 'detectionfactorFR'
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Flow deck and Multiranger deck verification
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        time.sleep(1)
        scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_multiranger)
        time.sleep(1)

        # Set the detection factor parameter
        simple_set_param_async(scf, group, name)

        # Logging variables details
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('kalman.stateF', 'float')
        logconf.add_variable('kalman.stateR', 'float')
        logconf.add_variable('kalman.tofdf', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # If the deck is not attached, then program exit
        if not flow_deck_attached_event.wait(timeout=5):
            print('No Flowdeck detected!')
            sys.exit(1)

        # If the deck is not attached, then program exit
        if not mr_deck_attached_event.wait(timeout=5):
            print('No multiranger deck detected!')
            sys.exit(1)

        # Start logging and drone movement
        logconf.start()
        check_test_f_r_move(scf)
        #time.sleep(5)
        # End logging
        logconf.stop()

        
