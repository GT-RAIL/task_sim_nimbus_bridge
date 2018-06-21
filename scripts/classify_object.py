#!/usr/bin/env python

# Python
import numpy as np
from sklearn.externals import joblib

# ROS
import rospy
import rospkg

from task_sim_nimbus_bridge.srv import Classify

class ObjectClassifier:

    def __init__(self):
        filename = rospy.get_param('~filename', 'random_forest.pkl')

        self.classifier = joblib.load(rospkg.RosPack().get_path('task_sim_nimbus_bridge') + '/data/' + filename)

        self.service = rospy.Service('~classify', Classify, self.perform_classification)

    def perform_classification(self, req):
        return self.classifier.predict(np.asarray(req.features).reshape(1, -1))[0]

if __name__ == '__main__':
    rospy.init_node('object_classifier')
    oc = ObjectClassifier()
    print 'Ready to classify objects.'

    rospy.spin()