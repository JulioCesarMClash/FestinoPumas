#! /usr/bin/env python
import rospy  
import rospkg
import yaml
import tf2_ros    
import tf
import math
import numpy as np
from std_msgs.msg import String                                # the main module for ROS-python programs
from known_locations_tf_server.srv import *
import pandas as pd
from geometry_msgs.msg import TransformStamped
from copy import deepcopy
global path 
def write_tf(pose, q, child_frame , parent_frame='map'):
    t= TransformStamped()
    t.header.stamp = rospy.Time(0)
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    #q = tf.transformations.quaternion_from_euler(eu[0], eu[1], eu[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
def np_to_str(np_arr):
    str_np=''
    for e in np_arr:
        str_np+= ','+str("%.3f" % e)
    return str_np

def read_tf(t):
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat

def read_yaml(known_locations_file = '/DepaDoc1.yaml'):
    
    file_path = '/home/robocup/FestinoPumas/PC_user/src/Navigation/config_files' + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

def write_yaml(trans,req, known_locations_file = '/DepaDoc1.yaml'):
    trans,quat=read_tf(trans)
    con= read_yaml()
    data=deepcopy(con[list(con.keys())[-1]])
    data[0]['x']=           math.trunc(trans[0]*1000)/1000
    data[1]['y']=           math.trunc(trans[1]  *1000)/1000
    data[2]['theta']=       math.trunc(tf.transformations.euler_from_quaternion(quat)[2]*1000)/1000
    data[3]['qx']=          math.trunc(quat[0]    *1000)/1000
    data[4]['qy']=          math.trunc(quat[1]*1000)/1000
    data[5]['qz']=          math.trunc(quat[2]*1000)/1000
    data[6]['qw']=          math.trunc(quat[3]*1000)/1000
    con[req.location_name.data]=data 
    
    file_path = rospack.get_path('config_files')  + known_locations_file

    print (con)
    with open(file_path, 'w') as file:
        documents = yaml.dump(con, file, default_flow_style=False)
    return True


def yaml_to_df():
    con = read_yaml()
    values=[]
    locations=[]
    for c in con:
        locations.append(c)

        for i in range(len(con[c])):
            values.append(list(con[c][i].values())[0])

    data=np.asarray(values).reshape((int(len(values)/7),7))    #x , y ,theta  ,quat   since z always 0
    df= pd.DataFrame( data)
    df.columns=['x','y','th','qx','qy','qz','qw']
    df['child_id_frame']=locations
    return df




###########################################################################################################################################################################
def callback(req):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    
    
    resp = Locations_serverResponse()
    #print (resp)
    try:
        trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        trans.child_frame_id= req.location_name.data
        tf_static_broadcaster.sendTransform(trans)
        
        
        
        #####################################
        
        succ=write_yaml(trans,req)
        print(succ)
        

        ###################################################
        #with  open(path , 'a') as out:
        #    out.write (req.location_name.data+np_to_str(trans)+np_to_str(quat)  +'\n' )
        #print (trans,quat)
        ####################### 


        resp.success.data= succ
        return resp




    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ( 'No TF FOUND')
        resp.success.data= False
        return resp

    

                 

    

global tfBuffer,tf_static_broadcaster ,known_locs , rospack
rospy.init_node('locs_server') 
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()
tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
rospack = rospkg.RosPack()


#df=pd.read_csv('known_locations.txt')

#df=pd.read_csv(path)##RELATIVIZE PATH?
df= yaml_to_df()
print (df)
known_locs=df.values
if len(df)!=0:
    for i in range(len(df)):
        trans=df[['x','y','th']].iloc[i].values
        quat=df[['qx','qy','qz','qw']].iloc[i].values
        t=write_tf(trans,quat,df['child_id_frame'].iloc[i])
        print (t,i)
        tf_static_broadcaster.sendTransform(t)
        rospy.sleep(0.3)


rospy.loginfo("known locations detection service available")                    # initialize a ROS node
rospy.Service('/knowledge/known_location_add', Locations_server, callback)        # type, and callback


rospy.spin()   
