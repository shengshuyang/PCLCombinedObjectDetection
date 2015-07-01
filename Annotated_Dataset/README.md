About USC Shadow
==========================
USC Shadow is a Point Cloud dataset of table top objects, with cast shadow labels.

How to Use:

--------------------------
The data:

The dataset currently contains 40 scenes, each scene comes with 3 files:

  *_downsampled.pcd
  
  *_rgb.png
  
  *_z.png

the PCD file is a point cloud collected using a Kinect sensor, with outlier points removed in a downsampling process. The two PNG files are the rgb and depth channel, respectively, converted from the PCD file with a tool provided by PCL.

--------------------------
The labels:

We provide a Matlab script named `label_usc_shadow.m`. By running it, the script will automatically download rgb channel images (*_rgb.jpg) and corresponding annotations from LabelMe server, then generate a set of masks, and plot one of the images overlayed with its shadow mask.

You'll need the LabelMe Matlab toolbox to run the script:

http://labelme.csail.mit.edu/Release3.0/browserTools/php/matlab_toolbox.php

Note that due to limitation of LabelMe, images pulled from LabelMe are in JPG format. 



 
