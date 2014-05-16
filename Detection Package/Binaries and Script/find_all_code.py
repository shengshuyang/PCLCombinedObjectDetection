#find all files in a directory

import os

output = open('output.txt','w+')

all_files = os.listdir('.')
for file in all_files:
  if '.h' in file or '.cpp' in file or '.hpp' in file:
    output.write(file + '\n')
	
output.close()

