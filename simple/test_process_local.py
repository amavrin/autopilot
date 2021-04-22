""" test process_local.py functions """

import process_local

def test_can_run_init():
    """ can run init function """
    process_local.init('process_local_test.conf')

def test_error_pause(capsys):
    """ test error_pause """
    process_local.error_pause("test", 0.1)
    captured = capsys.readouterr()
    assert captured.out == "ERROR: test\n"
