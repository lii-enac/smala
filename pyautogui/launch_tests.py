#!/usr/local/bin/python3

import os
import sys, getopt
import replay_autogui

test_list = [
  "dnd", \
  "bindings", \
  "clone"
]

def main(argv):

    _is_retina = ""
    _result_OK = []
    _result_KO = []

    # getopt
    try:
      opts, args = getopt.getopt(argv,"ha",["retina"])
    except getopt.GetoptError:
      print ('launch_tests.py -r|--retina')
      sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
          print ('launch_tests.py -r|--retina')
          sys.exit()
        elif opt in ("-r", "--retina"):
          _is_retina = "--retina"

    print ("\n\n------------------- ALL TESTS ----------------------")
    print ('Retina display:\t\t', _is_retina, '\n') 

    print ('Erasing: ALL results and diffs ')
    os.system ('rm -rf results/*.bmp')
    os.system ('rm -rf results/diffs/*.bmp')

    for itest in test_list:
      print ('\n\nlaunching test ... ', itest)
      result = replay_autogui.main (["-i", itest, _is_retina])
      if result:
        _result_OK.append (itest)
      else:
        _result_KO.append (itest)

    print ("\n\n \033[0;32m------------- ALL TESTS RESULTS ---------------------- \033[0m")
    for i in _result_OK:
      print ("\033[0;32m [OK] \033[0m - ", i)
    for i in _result_KO:
      print ("\033[0;31m [KO] \033[0m - ", i)

    print ("\n\n")
    #if result_KO not empty test if bad
    if not _result_KO == []:
      sys.exit(1) # for jenkin 


if __name__ == "__main__":
   main(sys.argv[1:])