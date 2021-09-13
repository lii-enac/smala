#!/usr/local/bin/python3

from pynput import keyboard
from pynput.mouse import Listener as MouseListener
from pynput.keyboard import Listener as KeyboardListener
from OSXwindow import find_window_geometry_by_name
import logging
import sys, getopt, subprocess

_is_ctrl_pressed = False
_screenshot_counter = 0
_app_name = "default"
_debug = False
_is_retina = False
_save_directory = "new_records/"

def on_key_press(key):
    global _is_ctrl_pressed, _debug, _is_retina
    if key == keyboard.Key.ctrl:
        _is_ctrl_pressed = True
        return True
    if key == keyboard.Key.alt:
        if _is_ctrl_pressed:
            import pyautogui
            global _screenshot_counter
            _screenshot_counter = _screenshot_counter + 1
            geo = find_window_geometry_by_name (_app_name + "_app")
            if (geo) == None:
                print ("\tWARNING - \'" + _app_name + "\' window NOT FOUND")
                mouse_listener.stop ()
                return False
            X = geo['X']
            Y = geo['Y']
            width = geo['Width']
            height = geo['Height']
            pos = pyautogui.position ()
            if _is_retina:
                retina_name = "_Retina"
                retina_coef = 2
            else:
                retina_name = ""
                retina_coef = 1
            screenshot_name = _save_directory + _app_name + "_" + str(_screenshot_counter) + retina_name + "_Ref.bmp"
            pyautogui.screenshot (screenshot_name, region=(X*retina_coef, Y*retina_coef, width*retina_coef, height*retina_coef))
            if _debug:
                print (screenshot_name, X*retina_coef, Y*retina_coef, width*retina_coef, height*retina_coef, pos.x, pos.y)
            logging.info('screenshot {0} {1} {2} {3} {4} {5} {6}'.format(_screenshot_counter, X, Y, width, height, pos.x, pos.y))
    else:
        if _debug:
            print(key)
        logging.info('k {0}'.format(key))
    return True
        
def on_key_release(key):
    global _is_ctrl_pressed, _debug
    if _debug:
        print("Key released: {0}".format(key))

    if key == keyboard.Key.ctrl:
        _is_ctrl_pressed = False
    if key == keyboard.Key.esc:
        mouse_listener.stop ()
        logging.info('quit x x')
        return False #terminate listener

def on_move(x, y):
    global _debug
    if _debug:
        print ("({0}, {1})".format(int (x), int(y)))
    logging.info('m {0} {1}'.format(int (x), int(y)))

def on_click(x, y, button, pressed):
    global _debug
    if pressed:
        if _debug:
            print ('PRESS at ({0}, {1}) with {2}'.format(int (x), int(y), button))  
        logging.info('p {0} {1} {2}'.format(int (x), int(y), button))
    else: #release
        if _debug:
            print ('RELEASE at ({0}, {1}) with {2}'.format(int (x), int(y), button))
        logging.info('r {0} {1} {2}'.format(int (x), int(y), button))

keyboard_listener = KeyboardListener(on_press=on_key_press, on_release=on_key_release)
mouse_listener = MouseListener(on_move=on_move, on_click=on_click)

def main(argv):
   
    global _debug, _app_name, _is_retina

    # getopt
    try:
      opts, args = getopt.getopt(argv,"hi:mr:",["itest=", "mode=", "retina="])
    except getopt.GetoptError:
      print ('replay_autogui.py -i|-itest <test_name> -m|--mode <debug> -r|--retina <true>')
      sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
          print ('replay_autogui.py -i|-itest <test_name> -m|--mode <debug> -r|--retina <true>')
          sys.exit()
        elif opt in ("-i", "--itest"):
          _app_name = arg
        elif opt in ("-m", "--mode"):
            if arg == 'debug':
                _debug = True
            else:
                _debug = False
        elif opt in ("-r", "--retina"):
            if arg == 'true':
                _is_retina = True
    datafile = _save_directory + _app_name + "_data.txt"

    # not working for second screen :/
    #if subprocess.call("system_profiler SPDisplaysDataType | grep -i 'retina'", shell=True) == 0:
    #    _is_retina = True

    print ("------------ RECORD -------------")
    print ('test name:\t', _app_name)
    print ('Data file:\t', datafile)
    print ('debug_mode:\t', _debug)
    print ('Retina display:\t', _is_retina, '\n')
    print ('WARNING:')
    print ('all files will be saved in : new_records directory')
    print ('You have to manually copy them if you want to use them\nwith ./replay_autogui.py\n\n')
    print ("Press CTRL+ALT to make screenshot")
    print ("Press ESC to Quit")
    print ("---------------------------------\n")
    
    logging.basicConfig(filename=datafile, filemode='w', level=logging.DEBUG, format='%(message)s')

    if _debug:
        print(">>>> starting recording >>>>")
    keyboard_listener.start()
    mouse_listener.start()
    if _debug:
        print("> listeners started")
        print("> waiting for keyboard to join")
    keyboard_listener.join()
    if _debug:
        print("> waiting for mouse to join")
    mouse_listener.join()
    if _debug:
        print("<<<< exiting recording <<<<<")
        

if __name__ == "__main__":
   main(sys.argv[1:])