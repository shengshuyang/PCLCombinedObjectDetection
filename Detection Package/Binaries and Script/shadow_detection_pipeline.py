import os
import sys
file = sys.argv[1]

#ret = os.system('DownSample '+file)

#ret = os.system('PlaneDetection '+'downsampled_'+file)

#ret = os.system('RegionGrowingHSV '+'hsv_downsampled_'+file)

ret = os.system('matlab -nosplash -nodesktop -wait -exit -r clusters_postprocess')
if ret != 0:
    print 'Error occurred!'
    sys.exit()

ret = os.system('VisualizeResults '+'hsv_downsampled_'+file)


#red for shadow
#green for non shadow
#blue for object
