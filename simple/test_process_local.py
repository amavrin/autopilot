""" test process_local.py functions """

import process_local

def test_error_pause(capsys):
    """ test error_pause """
    process_local.error_pause("test", 0.1)
    captured = capsys.readouterr()
    assert captured.out == "ERROR: test\n"
