from pyjoystick.sdl2 import Key, Joystick, run_event_loop

def print_add(joy):
    print('Added', joy)

def print_remove(joy):
    print('Removed', joy)

def key_received(key):
    print('Key:', key)
    if key.keytype == Key.BUTTON and key.number == 0:
        if key.value == 1:
            # Button 0 pressed
            print("Do action!")
        else:
            # Button 0 released
            pass

run_event_loop(print_add, print_remove, key_received)