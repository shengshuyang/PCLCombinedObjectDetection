import os
import Image
for root, dirs, files in os.walk(".\USC_Table_Top", topdown=False):
  for name in files:
    if 'rgb.png' in name:
      print name
      im = Image.open(os.path.join(root, name))
      im.save(os.path.join('.\USC_Table_Top_JPG', name.strip('.png')+'.jpg'))