""" test process_local.py functions """

from process_local import *

def test_error_pause(capsys):
    """ test error_pause """
    error_pause("test", 0.1)
    captured = capsys.readouterr()
    assert captured.out == "ERROR: test\n"
