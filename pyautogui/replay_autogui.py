#!/usr/local/bin/python3

from os import remove
import sys, getopt, subprocess
import pyautogui
from PIL import Image
from PIL import ImageChops, ImageStat
#pyautogui.FAILSAFE = True

#global variables
_screenshot_counter = 0
_datafile = 'defautl'
_interpolate_mode = False
_app_name = 'default'
_is_pressed = False
_is_retina = False
_max_screenshot = 0

def fct_move (line, button):
  global _interpolate_mode, _is_pressed
  if _interpolate_mode == False:
    xOff = float (line.split(' ')[1])
    yOff = float (line.split(' ')[2])
    #print (type, xOff, yOff)
    #button = line.split(' ')[5]
    if _is_pressed == True:
      pyautogui.dragTo (xOff, yOff, button=button, mouseDownUp=False)
    else:
      pyautogui.moveTo (xOff, yOff)


def fct_press (line, button):
  global _interpolate_mode, _is_pressed
  xOff = float (line.split(' ')[1])
  yOff = float (line.split(' ')[2])
  #button = line.split(' ')[3]
  #if _interpolate_mode == False:     
  _is_pressed = True
  pyautogui.mouseDown (xOff, yOff, button)
  #else:
  if _interpolate_mode == True:
    pyautogui.moveTo (xOff, yOff, duration=1, tween=pyautogui.easeOutQuad)


def fct_release (line, button):
  global _interpolate_mode, _is_pressed
  xOff = float (line.split(' ')[1])
  yOff = float (line.split(' ')[2])
  #button = line.split(' ')[3]
  if _interpolate_mode == False:
    _is_pressed = False
    pyautogui.mouseUp (xOff, yOff, button)
  else:
    pyautogui.dragTo (xOff, yOff, button=button, duration=1, tween=pyautogui.easeOutQuad)

def fct_key_press (line):
  print (line)

def fct_compare_images ():
  global _is_retina, _app_name, _max_screenshot
  if _is_retina:
    retina_name = '_Retina'
  else:
    retina_name =''

  # TODO remove
  _max_screenshot = 3

  for i in range(1, int(_max_screenshot)+1):
    im1_name = _app_name + "_"+ str(i) + retina_name + "_Ref.bmp"
    im2_name = _app_name + "_"+ str(i) + retina_name + ".bmp"
    im1 = Image.open(im1_name)
    im2 = Image.open(im2_name)
    
    if (im1.mode != im2.mode) \
            or (im1.size != im2.size) \
            or (im1.getbands() != im2.getbands()):    
        print ("saome wrong !!")
        return None

    diff = ImageChops.difference(im1, im2)
    stat = ImageStat.Stat(diff)
    diff_ratio = sum(stat.mean) / (len(stat.mean) * 255) * 100
    print ("diff_ratio: ", diff_ratio)

    #if diff.getbbox() != None:
    if diff_ratio > 0.05:
      diff_name = _app_name + "_" + str(i) + retina_name + "_DIFF.bmp"
      diff.save(diff_name)
      print("WARNING --- images are DIFFERENT - those differences has been saved in:", diff_name)
    else:
      print ("OK - ", im1_name, " - ", im2_name)

def fct_quit ():
  fct_compare_images ()
  sys.exit ()

def fct_screenshot (line):
  global _app_name, _max_screenshot_, _is_retina
  global _interpolate_mode, _max_screenshot, _is_pressed
  if _is_retina:
    retina_name = "_Retina"
    retina_coef = 2
  else:
    retina_name = ""
    retina_coef = 1

  _max_screenshot = line.split(' ')[1]
  x = float (line.split(' ')[2])
  y = float (line.split(' ')[3])
  width = float (line.split(' ')[4])
  height = float (line.split(' ')[5])
  mouse_x = float (line.split(' ')[6]) 
  mouse_y = float (line.split(' ')[7]) 
  #we have to move the mouse anyaway on _interpolated_mode to get object a the right place
  if _interpolate_mode:
    if _is_pressed == True:
      #TODO : change button
      pyautogui.dragTo (mouse_x, mouse_y, button='left', mouseDownUp=False, duration=1)
    else:
      pyautogui.moveTo (mouse_x, mouse_y, duration=1)

  #print (_max_screenshot, x, y, width, height, mouse_x, mouse_y)
  screenshot_name = _app_name + "_"+ _max_screenshot + retina_name + ".bmp"
  #print (screenshot_name)
  pyautogui.screenshot (screenshot_name, region=(x*retina_coef, y*retina_coef, width*retina_coef, height*retina_coef))


def main(argv):
    global _interpolate_mode, _datafile, _app_name, _is_retina

    # getopt
    try:
      opts, args = getopt.getopt(argv,"hi:m:",["itest=", "mode="])
    except getopt.GetoptError:
      print ('replay_autogui.py -i <test_name> -m <interpolate>')
      sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
          print ('replay_autogui.py -i <test_name> -m <interpolate>')
          sys.exit()
        elif opt in ("-i", "--itest"):
          _app_name = arg
        elif opt in ("-m", "--mode"):
          if arg == "interpolate":
            _interpolate_mode = True
          else:
            _interpolate_mode = False
    _datafile = _app_name + "_data.txt"
    
    if subprocess.call("system_profiler SPDisplaysDataType | grep -i 'retina'", shell=True) == 0:
        _is_retina = True

    print ("------------ REPLAY -------------")
    print ('test name:\t', _app_name)
    print ('Data file:\t', _datafile)
    print ('interpolate_mode:\t', _interpolate_mode)
    print ('Retina display:\t', _is_retina, '\n')
    print ("---------------------------------\n")

    #file parsing
    #_is_pressed = False
    with open(_datafile) as data:
        for line in data:
            line.rstrip('\n')
            #print(line)
            type = line.split(' ')[0]

            button = 'left'

            if type == 'm':
              fct_move (line, button)

            if type == "p" :
              fct_press (line, button)

            if type == "r" :
              fct_release (line, button)
            
            if type == "k" :
              fct_key_press (line)
            
            if type == "quit" :
              fct_quit ()

            if type == "screenshot" :
              fct_screenshot (line)
    data.close ()

if __name__ == "__main__":
   main(sys.argv[1:])