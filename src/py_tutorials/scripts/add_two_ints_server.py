#!/usr/bin/env python

NAME = 'add_two_ints_server'
from py_tutorials.srv import *
import rospy

def add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    sum = req.a + req.b
    return AddTwoIntsResponse(sum)

def add_two_ints_server():
    rospy.init_node(NAME)
    s=rospy.Service('add_two_ints',AddTwoInts,add_two_ints)
    print "Ready to add two ints"
    rospy.spin()

if __name__=="__main__":
    add_two_ints_server()

