#!python3

import os
import sys, getopt, subprocess, time
import pyautogui
from PIL import Image
from PIL import ImageChops, ImageStat
pyautogui.FAILSAFE = True

#global variables
_datafile = 'defautl'
_interpolate_mode = True
_app_name = 'default'
_is_pressed = False
_is_retina = False
_max_screenshot = 0
_data_dir = "datas/"
_ref_dir = "references/"
_result_dir = "results/"
_diff_dir = _result_dir + "diffs/"
_RATIO = 1
_time_to_wait = 2

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
  _is_pressed = True
  pyautogui.mouseDown (xOff, yOff, button)
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
  global _ref_dir, _result_dir, _diff_dir, _RATIO
  if _is_retina:
    retina_name = '_Retina'
  else:
    retina_name =''

  result = True;
  for i in range(1, int(_max_screenshot)+1):
    im1_name = _ref_dir + _app_name + "_"+ str(i) + retina_name + "_Ref.bmp"
    im2_name = _result_dir + _app_name + "_"+ str(i) + retina_name + ".bmp"
    im1 = Image.open(im1_name)
    im2 = Image.open(im2_name)
    
    diff = ImageChops.difference(im1, im2)
    stat = ImageStat.Stat(diff)
    diff_ratio = sum(stat.mean) / (len(stat.mean) * 255) * 100
    print ("diff_ratio: ", diff_ratio)

    #TODO: return False or True ?
    #if diff.getbbox() != None:
    if diff_ratio > _RATIO:
      diff_name = _diff_dir + _app_name + "_" + str(i) + retina_name + "_DIFF.bmp"
      diff.save(diff_name)
      print("\033[0;31m WARNING --- images are DIFFERENT - those differences has been saved in:", diff_name, "\033[0m")
      result = False
    else:
      print ("\033[0;32m [OK] - ", im1_name, " - ", im2_name, "\033[0m")

  return result

def fct_quit ():
  result = fct_compare_images ()
  return result
  # if result == False:
  #   sys.exit(1)
  # sys.exit (0)

def fct_screenshot (line):
  global _app_name, _max_screenshot_, _is_retina
  global _interpolate_mode, _max_screenshot, _is_pressed, _result_dir
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
  if _interpolate_mode == True:
    if _is_pressed == True:
      #TODO : change button
      pyautogui.dragTo (mouse_x, mouse_y, button='left', mouseDownUp=False, duration=1)
    else:
      pyautogui.moveTo (mouse_x, mouse_y, duration=1)

  #print (_max_screenshot, x, y, width, height, mouse_x, mouse_y)
  screenshot_name = _result_dir + _app_name + "_"+ _max_screenshot + retina_name + ".bmp"
  #print (screenshot_name)
  pyautogui.screenshot (screenshot_name, region=(x*retina_coef, y*retina_coef, width*retina_coef, height*retina_coef))


def main(argv):
    global _interpolate_mode, _datafile, _app_name, _is_retina, _data_dir, _ref_dir
    global _time_to_wait

    # getopt
    try:
      opts, args = getopt.getopt(argv,"hi:trn",["itest=", "test", "retina", "no_interpolation"])
    except getopt.GetoptError:
      print ('replay_autogui.py -i <test_name> -t|--test -r|--retina -n|--no_interpolation')
      sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
          print ('replay_autogui.py -i <test_name> -t|--test -r|--retina -n|--no_interpolation')
          sys.exit()
        elif opt in ("-i", "--itest"):
          _app_name = arg
        elif opt in ("-t", "--test"):
          _data_dir = "new_records/"
          _ref_dir = "new_records/"
        elif opt in ("-n", "--no_interpolation"):
          _interpolate_mode = False
        elif opt in ("-r", "--retina"):
          _is_retina = True
    _datafile = _data_dir + _app_name + "_data.txt"

    print ("\n------------ REPLAY ", _app_name ," -------------")
    print ('test name:\t\t', _app_name)
    print ('Data file:\t\t', _datafile)
    print ('interpolation:\t\t', _interpolate_mode)
    print ('Retina display:\t\t', _is_retina, '\n')

    print ('Erasing: ', _app_name, ' results and diffs ')
    os.system ('rm -rf results/' + _app_name + '*.bmp')
    os.system ('rm -rf results/diffs/' + _app_name + '*.bmp')
    
    print ('lauching test:', _app_name + "_test")
    subprocess.Popen([ "make", "-j", _app_name + "_test"], cwd="..", stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep (_time_to_wait) #wait for a second that the app launch 

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
              return fct_quit ()

            if type == "screenshot" :
              fct_screenshot (line)
    data.close ()

if __name__ == "__main__":
   main(sys.argv[1:])