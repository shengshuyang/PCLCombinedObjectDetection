import os
import sys

names = open('pcd_names.txt')
file_name = 'start'
#resolution = [0.008,0.010,0.012,0.014,0.020]
#resolution = [0.008,0.010]
resolution = [0.004]
res = 0.004
ret = 0
while file_name:
  file_name = names.readline().strip('\n').replace('.pcd','')
  print file_name
  # if file_name != 'Dataset2\\pos3\\box_pos3_dark_1':
    # continue
  for res in resolution:
    ret = os.system('DownSample '+file_name+'.pcd' + ' ' + str(res) + ' ' + str(res) + ' ' + str(res))
    if ret != 0:
      print '\nDownsampling failed, check filename!'
      sys.exit()
      continue
      
    ret = os.system('PlaneDetection '+file_name+'_downsampled.pcd')
    if ret != 0:
      print '\nPlane Detection failed, check filename!'
      sys.exit()
      continue

    ret = os.system('RegionGrowingHSV '+file_name+'_downsampled_hsv.pcd')
    if ret != 0:
      print '\nRegionGrowingHSV failed, check filename!'
      sys.exit()
      continue

    ret = os.system('matlab -nosplash -nodesktop -wait -exit -r clusters_postprocess')
    if ret != 0:
      print 'Matlab script error occurred!'
      sys.exit()
      continue

    ret = os.system('VisualizeResults '+file_name+'_downsampled_hsv.pcd' + ' ' + str(int(1000*res)))
    if ret != 0:
      print 'Visualize Results failed!'
      sys.exit()
      continue

    # ret = os.system('LightDetection '+file_name+'_downsampled_hsv_' + str(int(1000*res)) + '_shadow_detected.pcd')
    # if ret != 0:
      # print 'LightDetection failed!'
      # sys.exit()
      # continue
#red for shadow
#green for non shadow
#blue for object
