""" import Python's standard time module """
import time

# import Controller and other blocks from modules
from pyctrl import Controller
from pyctrl.block import Printer
from fg_source import FgSource
from fg_sink import FgSink

pilot = Controller()

pilot.add_signal('speed')
pilot.add_signal('heading')
pilot.add_signal('altitude')
pilot.add_signal('climb')
pilot.add_signal('pitch')
pilot.add_signal('bank')
pilot.add_signal('engine')
pilot.add_signal('latitude')
pilot.add_signal('longitude')
pilot.add_signal('northspeed')
pilot.add_signal('eastspeed')

pilot.add_signal('aileron')
pilot.add_signal('elevator')
pilot.add_signal('rudder')
pilot.add_signal('throttle')

pilot.add_source('fg_source',
                 FgSource(),
                 ['speed', 'heading', 'altitude', 'climb', 'pitch', 'bank',
                  'engine', 'latitude', 'longitude', 'northspeed', 'eastspeed'],
                 enable = True)

pilot.add_sink('debug_print',
               Printer(message = 'speed: {}, heading: {}'),
               ['speed', 'heading'],
               enable = True)

pilot.add_sink('fg_sender',
               FgSink(),
               ['aileron', 'elevator', 'rudder', 'throttle'],
               enable = True)

try:
    # run the controller
    with pilot:
        # do nothing for 5 seconds
        while True:
            time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    print('Done')
