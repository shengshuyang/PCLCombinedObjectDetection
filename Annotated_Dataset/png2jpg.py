import os
from PIL import Image
for root, dirs, files in os.walk(".\SelectedNYU", topdown=False):
  for name in files:
    if 'rgb.png' in name:
      print name
      im = Image.open(os.path.join(root, name))
      im.save(os.path.join('.\SelectedNYU_JPG', name.strip('.png')+'.jpg'))