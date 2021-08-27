# copyright (c) ENAC 2018

from PIL import Image, ImageOps, ImageChops
from glob import glob
from shutil import copy2
import os

# some refs
# https://pillow.readthedocs.io/en/5.1.x/
# https://codereview.stackexchange.com/questions/55902/fastest-way-to-count-non-zero-pixels-using-python-and-pillow

src_dir_name = "shots"
dst_dir_name = "anims"


# first remove previous results
for f in glob(os.path.join(dst_dir_name,'*')):
    os.remove(f)

def load_and_process(filename):
    img = Image.open(filename) #.convert("L")
    return img

num_in=1
num_out=0

filenames = glob(os.path.join(src_dir_name,'*.png'))
filenames = sorted(filenames)

tocmp = load_and_process(os.path.join(filenames[0]))
copy2(filenames[0], dst_dir_name)

def build_blended_name_from_source_name (filename):
    return os.path.join(dst_dir_name, os.path.basename(filename))

for filename in filenames[1:]:
    print (filename)
    img = load_and_process(filename)
    blended = Image.blend(tocmp, img, alpha=0.6)
    blended_name = build_blended_name_from_source_name (filename)
    blended.save(blended_name)

    #tocmp = img
    tocmp = blended
    num_out += 1
    num_in += 1

print("processed "+str(num_out)+"/"+str(num_in))
