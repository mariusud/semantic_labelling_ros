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
import subprocess
import threading
import psutil


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()

sequence_incrementer = 0
cwd = os.getcwd()
print(os.path.join(cwd, "/stairs"))
os.chdir(cwd + '/stairs')

for obj in glob.glob("*.dae"):
    #time.sleep(20) # cooldown pause between runs
    print("RUN number %d \n \n" %  sequence_incrementer)
    # Folder creations
    sequence_path = cwd + '/dataset/' + str(sequence_incrementer)
    sequence_ouster_path = os.path.join(sequence_path, 'velodyne')
    sequence_label_path = os.path.join(sequence_path, 'labels')

    # make txt with filename etcs
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
    except FileExistsError:
        pass

    # Main processing line
    proc = subprocess.Popen(['roslaunch', '/home/marius/Development/SemanticSegmentation/launch/segmentation.launch'])
    #labelnode = LabellingNode()
    #label_thread = threading.Thread(target=labelnode.start(sequence_path), args=(1,))
    #label_thread.start()
    #writer = threading.Thread(target=DatasetWriter(sequence_path), args=(1,))
    #writer.start()

    # ADD STAIRS

    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        kill(proc.pid)
    #rospy.signal_shutdown('reason') 


    sequence_incrementer += 1
    time.sleep(20)
