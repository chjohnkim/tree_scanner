#!/usr/bin/env python
import rospy
import subprocess
import os
import signal


class RosbagRecord:
    def __init__(self, record_string, record_folder):
        self.record_string = record_string
        self.record_folder = record_folder
        # Start recording.
        command = record_string + ' -o ' + record_folder 
        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True,# cwd=self.record_folder,
                executable='/bin/bash')
        # Wait for shutdown signal to close rosbag record

    def _stop_record(self):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % self.p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        self.p.terminate()
        
    def stop_record(self):
        import psutil
        process = psutil.Process(self.p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        self.p.wait()  # we wait for children to terminate

if __name__ == '__main__':

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rosbag_record = RosbagRecord('rosbag record tf', './')
        rospy.sleep(5)
        rosbag_record.stop_record()
        rospy.sleep(1)
    except Exception as e:
        print(e)

