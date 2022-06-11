'''!
    @file       main.py
    
    @brief      This is the main program that connects all the different tasks together
    
    @details    This utilizes the cotask to create a cooperative set of tasks for simulaneous 
                actions. This means it takes in the usesr_input task, the process_hpgl task, 
                and also the positioning task.
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''

from task import task_share, cotask

from UserInput import MAX_FILENAME, task_user_input
from ProcessesHPGL import task_process_hpgl
from Positioning import task_positioning

import micropython, pyb

micropython.alloc_emergency_exception_buf(100)
pyb.repl_uart(None)

filename = task_share.Queue('B', MAX_FILENAME)

# todo: overwrite True vs clear then write
polar_angle = task_share.Queue('f', 1)
azimuthal_angle = task_share.Queue('f', 1)
fire = task_share.Queue('B', 1)

paused = task_share.Share('B')
stopped = task_share.Share('B')
stopped.put(1)


def main(): 
    '''!@brief      This function uses cotask to link all the tasks together. 
    '''
    user_input_task = cotask.Task(task_user_input((polar_angle, azimuthal_angle, fire), filename, paused, stopped), 'User Input Task', 1, 4, True, False)
    process_hpgl_task = cotask.Task(task_process_hpgl(filename, (polar_angle, azimuthal_angle, fire)), 'Process HPGL Task', 1, 4, True, False)
    positioning_task = cotask.Task(task_positioning((polar_angle, azimuthal_angle, fire), paused, stopped), 'Positioning Task', 1, 4, True, False)

    cotask.task_list.append(user_input_task)
    cotask.task_list.append(process_hpgl_task)
    cotask.task_list.append(positioning_task)

    while True:
        # if paused, only schedule user input task
        if paused.get():
            user_input_task.schedule()
        # otherwise, do priority scheduling
        else:
            cotask.task_list.pri_sched()


if __name__ == '__main__':
    main()
