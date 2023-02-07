#!/usr/bin/env python
import rospy
import random
import numpy as np
import datetime
from std_msgs.msg import String

def ordergen():
    #Order 1: 1 x C0 from 00:00 to 17:00
    order_arr = np.zeros([10,3])
    for i in range(9):
        order_num = i
        order_cmplx = random.randint(1,2)
        order_type = random.randint(0,3)
        order_arr[i,0] = order_num+1
        order_arr[i,1] = order_cmplx
        order_arr[i,2] = order_type
    return order_arr

def mps_state_gen():
    global rate
    machine_types = ['CS','RS','DS','SS','BS']
    mps_type = machine_types[random.randint(0,len(machine_types))]
    mps_num = random.randint(1,2)
    fail_time = random.randint(0,10)
    mps_names = ['M-'+mps_type+str(int(mps_num)),'C-'+mps_type+str(int(mps_num))] 
    state_msg = 'Machine '+mps_names[0]+' down for '+str(int(fail_time))+' sec \nMachine '+mps_names[1]+' down for '+str(int(fail_time))+' sec'
    print(state_msg)
    rospy.sleep(fail_time)
    state_msg = 'Machine '+mps_names[0]+' is up again \nMachine '+mps_names[1]+' is up again'
    print(state_msg)


def rb_msg_pub(pub, rb_msg):
    print(rb_msg)
    pub.publish(rb_msg)

def main():
    rospy.init_node('fake_refbox', anonymous=True)
    global rate
    rate = rospy.Rate(5)
    pub = rospy.Publisher('/refbox_msg', String, queue_size=10)
    used_indx_order = np.array([])
    order_arr = ordergen()
    mps_state_gen()
    while not rospy.is_shutdown():
        order_indx = random.randint(0,order_arr.shape[0]) -1
        if not (order_indx in used_indx_order) and order_indx != -1:
            order_msg = 'Order '+str(int(order_arr[order_indx,0]))+': '+str(int(order_arr[order_indx,1]))+' x C'+str(int(order_arr[order_indx,2]))
            used_indx_order = np.append(used_indx_order, [int(order_indx)])
            rb_msg_pub(pub, order_msg)
            if used_indx_order.shape[0] >= 10:
                print("All orders published")
        rate.sleep()



if __name__ == '__main__':
    main()