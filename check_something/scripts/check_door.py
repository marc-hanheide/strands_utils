#!/usr/bin/env python
import rospy
import actionlib
from check_something.checker import Checker
from strands_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from ros_mary_tts.msg import *
from ros_datacentre.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
from sensor_msgs.msg import Image

class DoorChecker(Checker):
    def __init__(self): 
        super( DoorChecker, self ).__init__('check_door', 'doorDetection', DoorCheckAction)
        self.speech_client = None
        # Used to create some blog entries
        blog_collection = 'robblog'
        self.blog_msg_store = MessageStoreProxy(collection=blog_collection)
        self.last_image = None

    def speak(self, text):
        if self.speech_client == None:
            action = 'speak'
            rospy.loginfo('waiting for %s server' % (action))
            self.speech_client = actionlib.SimpleActionClient(action, maryttsAction)
            self.speech_client.wait_for_server(rospy.Duration(10))

        if self.speech_client != None:
            goal = maryttsGoal(text)
            self.speech_client.send_goal_and_wait(goal, rospy.Duration(20))

        rospy.loginfo(text)

        
    def img_callback(self, img):
        self.last_image = img

    def generate_post(self, door_open):

        # grab an image
        self.last_image = None
        image_topic = '/chest_xtion/rgb/image_color'
        image_sub = rospy.Subscriber(image_topic, Image, self.img_callback)

        count = 0
        while self.last_image == None and  not rospy.is_shutdown() and count < 10:
            rospy.loginfo('waiting for image %s' % count)
            count += 1
            rospy.sleep(1)

        e = RobblogEntry(title='Door Check ' + str(rospy.get_rostime().secs))
        e.body = 'I checked the door and it was '
        if door_open:
            e.body += '*open*'
        else:
            e.body += '*closed*'
        e.body += '.'

        if self.last_image != None:
            img_id = self.blog_msg_store.insert(self.last_image)
            rospy.loginfo('adding image to blog post')
            e.body += '\n\n![Image of the door](ObjectID(%s))' % img_id

        self.blog_msg_store.insert(e)

    def generate_report(self, state, result):   


        if state == GoalStatus.SUCCEEDED:
            if result.open:
                self.speak("I think the door is open")
            else:
                self.speak("I think the door is closed")
            self.generate_post(result.open)
        else:   
            rospy.loginfo("Action interrupted or aborted")


if __name__ == '__main__':
    rospy.init_node("door_checker")
    checker = DoorChecker()  
    rospy.spin()

   
