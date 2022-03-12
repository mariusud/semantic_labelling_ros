#!/usr/bin/env python3.8

## todo
## iterate thourgh all stair files
## create a sequence for each
## run launchfile & start the running
## add the correct stair
## start listener and writer
## end after 30 mins
## start new one
import listener
from listener import LabellingNode
from writer import DatasetWriter
import glob, os, time

sequence_incrementer = 0
cwd = os.getcwd()
print(cwd + "/stairs")
print(os.path.join(cwd, "/stairs"))
os.chdir(cwd + '/stairs')

for obj in glob.glob("*.dae"):
    #time.sleep(20) # cooldown pause between runs
    
    # Folder creations
    sequence_path = cwd + '/dataset/' + str(sequence_incrementer)
    sequence_ouster_path = os.path.join(sequence_path, 'ouster')
    sequence_label_path = os.path.join(sequence_path, 'labels')

    os.mkdir(sequence_path)
    os.mkdir(sequence_ouster_path)
    os.mkdir(sequence_label_path)


    # Main processing line
    #DatasetWriter(sequence_path)

    #subprocess.call launch add stairs using obj
    
    sequence_incrementer += 1
    #time.sleep(1800)
