#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests
import time

def fetch_thingspeak_data(channel_id, read_api_key, distance_pub, collision_pub):
    url = f"https://api.thingspeak.com/channels/yourchannelnumberhere/feeds.json?api_key=yourapikeyhere"
    
    try:
        while not rospy.is_shutdown():
            response = requests.get(url)
            if response.status_code == 200:
                data = response.json()
                feeds = data['feeds']
                for feed in feeds:
                    distance = float(feed.get('field1'))
                    rospy.loginfo("Distance: %d cm", distance)
                    distance_pub.publish(str(distance))  # Publish distance data
                    if distance < 10:
                        collision_msg = "Collision ahead! Distance: {} cm".format(distance)
                        rospy.logwarn(collision_msg)
                        collision_pub.publish(collision_msg)  # Publish collision message
            else:
                rospy.logerr("Failed to fetch data from ThingSpeak. Status code: %d", response.status_code)
            rospy.sleep(5)  # Fetch data every 5 seconds
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

def main():
    rospy.init_node('My_Robot')
    
    # Replace 'YOUR_THINGSPEAK_CHANNEL_ID' and 'YOUR_THINGSPEAK_READ_API_KEY' with your actual ThingSpeak channel ID and read API key if it works then ok else no need to replace anything below
    channel_id = 'YOUR_THINGSPEAK_CHANNEL_ID'
    read_api_key = 'YOUR_THINGSPEAK_READ_API_KEY'
    
    distance_pub = rospy.Publisher('distance', String, queue_size=10)
    collision_pub = rospy.Publisher('collision', String, queue_size=10)

    fetch_thingspeak_data(channel_id, read_api_key, distance_pub, collision_pub)

if __name__ == "__main__":
    main()
