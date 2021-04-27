""" test process_local.py functions """

import math
import process_local

def test_can_run_init():
    """ can run init function """
    process_local.init('process_local_test.conf')

def test_error_pause(capsys):
    """ test error_pause """
    process_local.error_pause("test", 0.1)
    captured = capsys.readouterr()
    assert captured.out == "ERROR: test\n"

def test_can_store_state_data():
    """ Set and retrieve state data """
    process_local.init('process_local_test.conf')
    assert process_local.get_state_data('nosuch') is None
    process_local.set_state_data('some', 'value')
    assert process_local.get_state_data('some') == 'value'
    process_local.next_state()
    assert process_local.get_state_data('some') is None
    process_local.set_state_data('some', 'other value')
    assert process_local.get_state_data('some') == 'other value'

def test_radius_calculus():
    """ test radius calculation """
    assert process_local.calculate_radius(1,0,100) is math.inf
    # An object moves at a constant linear speed of 10 m/sec
    # around a circle of radius 400 m. A central angle does sweeps in .0125 rads
    # in in 0.5 seconds.
    delta_head = math.degrees(.0125)
    radius = process_local.calculate_radius(0.5, delta_head,
                process_local.mps_to_knot(10))
    assert 399 < radius < 401
    radius = process_local.calculate_radius(0.5, -delta_head,
                process_local.mps_to_knot(10))
    assert -399 > radius > -401
