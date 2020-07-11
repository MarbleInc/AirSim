#!/usr/bin/env python
import rospy
import os
import json
from std_msgs.msg import String
from mbot_base.msg import TrackedObject, TrackedObjectArray
dirname=""

def trackedObjectCallback(tracks_msg):
    ''' parse Tracked Object Array and output to a file '''
    global dirname
    
    timestamp_ = "{:.3f}".format(tracks_msg.header.stamp.to_sec())
    filename = timestamp_ + ".json"

    result = {
        'frame': tracks_msg.header.frame_id,
        'timestamp': float(timestamp_),
        'device_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'device_heading': {
            'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0
        },
        'objects': []
    }
    for track in tracks_msg.tracks:
        obj = {
            'id': track.id,
            'center': {'x': track.position.x, 'y': track.position.y, 'z': track.position.z},
            'radius': track.radius,
            'velocity': {'x': track.velocity.x, 'y': track.velocity.y, 'z': track.velocity.z},
            'yaw_rate': track.yaw_rate,
            'orientation_known': track.orientation_known,
            'shape': {
                'height': track.shape.height,
                'footprint': []
            },
            'classification': track.classification
        }
        for point in track.shape.footprint:
            obj['shape']['footprint'].append({'x': point.x, 'y': point.y, 'z': point.z})

        result['objects'].append(obj)

    with open(os.path.join(dirname, filename), 'w') as fileobj:
        rospy.loginfo("writing tracked object file {}".format(os.path.abspath(os.path.join(dirname, filename))))
        json.dump(result, fileobj)  

def listener():
    global dirname
    rospy.loginfo("Json writer creating")
    rospy.init_node('json_writer', anonymous=True)

    # get target directory name
    param_name="/ground_truth_dirname"
    if rospy.has_param(param_name):
        dirname = rospy.get_param(param_name)
    else:
        rospy.logerr("{} param not set".format(param_name))
        exit(-1)
    
    # create target directory if it doesn't exist
    dirname = os.path.join(os.getcwd(), dirname)
    if not os.path.exists(dirname):
        os.mkdir(dirname)

    rospy.Subscriber("tracked_objects_simulated", TrackedObjectArray, trackedObjectCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    