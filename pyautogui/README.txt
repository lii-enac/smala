



0 - Intall pyautogui
___________________________________

site: https://pyautogui.readthedocs.io/en/latest/install.html

pip3 install pyautogui

pip3 install Pillow



On Linux, additionally you need to install the scrot application, as well as Tkinter:

sudo apt-get install scrot

sudo apt-get install python3-tk

sudo apt-get install python3-dev




1 - How to use
___________________________________

FROM smala/pyautogui/ directory:


- record_autogui.py script : 
cmd: autogui.py -i|-itest <test_name> -d|--debug -r|--retina
    -i : test to use without "_test"
    -d : verbose debug informations
    -r : use retina resolution

exemple: ./record_autogui.py -i simplest -d -r


- replay_autogui.py script :
cmd: replay_autogui.py -i <test_name> -t|--test -r|--retina -n|--no_interpolation
    -i : test to use without "_test"
    -t : test_mode to use fresh record from new_records/ directory otherwise it'll use datas/ directory
    -r : use retina resolution
    -n : do not use mouse move interpolation ... really precise BUT really slow

- launch_tests.py :