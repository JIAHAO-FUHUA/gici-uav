#!/usr/bin/python
# usage: python2 bag_batch_process.py --yaml=xxx.yaml --option=filter_bag

import os
import argparse
import yaml
import rosbag
import rospy
import subprocess
import psutil

plat_data_pair_list = [
    ('polyu', 'x_building'),
    ('polyu', 'u_garden')
    # ('hkust', 'campus_day_ouster128')
    # ('hilti', 'exp01'),
    # ('hilti', 'exp02'),
    # ('handheld', 'exp04'),
    # ('handheld', 'exp05'),
    # ('handheld', 'exp06')
    # ('hilti', 'exp07'),
    # ('hilti', 'exp09')
    # ('hilti', 'exp11'),
    # ('hilti', 'exp15')
    # ('hilti', 'exp21')
]

bag_path_download = '/home/aae/projects/gici-open/data/20240112-taipei/3/resource_analysis/'
bag_path_list = [
    bag_path_download + 'xsens_imu_1.bag',
    bag_path_download + 'xsens_imu_2.bag'
    # bag_path_download + '20220226_campus_road_day_ref.bag'
    # bag_path_download + 'exp01_construction_ground_level.bag',
    # bag_path_download + 'exp02_construction_multilevel.bag',
    # bag_path_download + 'exp_04_construction_upper_level_easy_2_2022-03-03-11-48-59.bag',
    # bag_path_download + 'exp_05_construction_upper_level_easy_2022-03-03-11-46-10.bag',
    # bag_path_download + 'exp_06_construction_upper_level_hard_2022-03-03-12-08-37.bag'
    # bag_path_download + 'exp07_long_corridor.bag',
    # bag_path_download + 'exp09_cupola.bag'
    # bag_path_download + 'exp11_lower_gallery.bag',
    # bag_path_download + 'exp15_attic_to_upper_gallery.bag'
    # bag_path_download + 'exp21_outside_building.bag'
]

algorithm_type_list = [
    # '0',  # liosam
    # '1',  # liomapping
    # '2'  # fastlio
    # '3',  # fasterlio
    # '4',  # liliom
    # '6'  # liosam_loop
    '7'  # fastlio_loop
]

test_times = 1
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    # source_command = 'source /media/aae/tommy/code/lidar/catkin_ws/devel/setup.bash'
    # os.system(source_command)

    # for i, pd_pair in enumerate(plat_data_pair_list):
    for  i in range(test_times):
        # bag_path_dir = bag_path_list[i]
        # for bag_path_dir in bag_path_list:
        for algorithm_type in algorithm_type_list:
            ################### run the algorithm
            # kill_command = 'rosnode kill gici'   
            # os.system(kill_command)   
            # rosnode_cleanup_command = 'rosnode cleanup'
            # os.system(rosnode_cleanup_command)
            # rospy.sleep(10)   # wait for the process to start
            # kill_command = 'rosnode kill gici'
            # os.system(kill_command)  
            # command = 'rosrun gici_ros gici_ros_main' + \
            #         ' /media/aae/tommy/code/gici/backup/20240121-rtk-ins-odom-ar/ros_wrapper/src/gici/option/ros_real_time_estimation_RTK_TC_taipei.yaml'
            # process = subprocess.Popen(command, shell=True)
            # pid = process.pid + 1   # the pid of the process is the pid of the shell, we should get the true pid of the process
            # print('pid: ', pid)
            pid = 17031
            # rospy.sleep(10)   # wait for the process to start

            ################## play the bag file
            # play_command = 'rosbag play ' + bag_path_dir + ' --clock --duration=5'
            play_command = 'rosbag play /home/aae/projects/gici-open/data/20240112-taipei/3/new_imu_odom.bag' + \
                            ' /home/aae/projects/gici-open/data/20240112-taipei/3/gnss_reference.bag ' + \
                            ' /home/aae/projects/gici-open/data/20240112-taipei/3/gnss_rover.bag' + \
                            ' -s 31738' + \
                            ' --clock --duration=1115'

            play_process = subprocess.Popen(play_command, shell=True)
            while(not (play_process.poll() is None)):
                rospy.sleep(1)   # wait for the process to start  (process.poll() is None and

            # rospy.sleep(3)
            ################# record the cpu and memory usage
            mkdir_command = 'mkdir -p ' + bag_path_download 

            cpu_file = bag_path_download +  '/cpu_usage'+ str(i) +'.txt'
            memory_file = bag_path_download +  '/memory_usage'+ str(i) +'.txt'
            with open(cpu_file ,'w') as cpu_f, open(memory_file, 'w') as memory_f:
                while(play_process.poll() is None):
                    try:
                        process_obj = psutil.Process(pid)
                        cpu_percent = process_obj.cpu_percent(interval=1)
                        memory_percent = process_obj.memory_percent()
                    except psutil.NoSuchProcess:
                        break
                    cpu_percent = process_obj.cpu_percent(interval=1)
                    memory_percent = process_obj.memory_percent()
                    print('cpu_percent: ', cpu_percent)
                    print('memory_percent: ', memory_percent)
                    cpu_f.write(str(cpu_percent) + '\n')
                    memory_f.write(str(memory_percent) + '\n')
                    # rospy.sleep(1) # check every 1 second
                # cpu_f.close()
                # memory_f.close()


            # process.terminate()
            # play_process.wait()
 



            # rospy.sleep(5)   # wait for the process to start
            # process.terminate()
            # kill_command = 'rosnode kill gici'
            # os.system(kill_command)            


            # rospy.sleep(3)   # wait for the process to start
                           
            rospy.sleep(10)   # wait for the process to start
            # print("================run algorithm at %s sequence==================" % (
            # print(' sequence:=' + pd_pair[1])
            # os.system(command)
