#!/usr/bin/env python3.8

## todo
## iterate thourgh all stair files
## create a sequence for each
## run launchfile & start the running
## add the correct stair
## start listener and writer
## end after 30 mins
## start new one
from listener import LabellingNode
from writer import DatasetWriter
import glob, os, time

sequence_incrementer = 0
cwd = os.getcwd()
print(os.path.join(cwd, "/stairs"))
os.chdir(cwd + '/stairs')

for obj in glob.glob("*.dae"):
    #time.sleep(20) # cooldown pause between runs
    
    # Folder creations
    sequence_path = cwd + '/dataset/' + str(sequence_incrementer)
    sequence_ouster_path = os.path.join(sequence_path, 'velodyne')
    sequence_label_path = os.path.join(sequence_path, 'labels')
    try:
        os.mkdir(sequence_path)
        os.mkdir(sequence_ouster_path)
        os.mkdir(sequence_label_path)
    except FileNotFoundError:
        print("Directory: {0} does not exist".format(sequence_path))
    except NotADirectoryError:
        print("{0} is not a directory".format(sequence_path))
    except PermissionError:
        print("You do not have permissions to change to {0}".format(sequence_path))



    # Main processing line
    #LabellingNode()
    DatasetWriter(sequence_path)

    #subprocess.call launch add stairs using obj
    
    sequence_incrementer += 1
    time.sleep(1800)
