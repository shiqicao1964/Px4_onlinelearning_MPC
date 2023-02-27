import rospy
from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandHomeResponse

# Initialize the ROS node
rospy.init_node('set_home_node')

# Wait for the mavros services to become available
rospy.wait_for_service('/mavros/cmd/set_home')

# Create a proxy for the set home service
set_home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

# Create a CommandHomeRequest object with the new home location
request = CommandHomeRequest()
request.latitude = 47.6199 # replace with your desired latitude
request.longitude = -122.3535 # replace with your desired longitude
request.altitude = 10.0 # replace with your desired altitude

# Call the set home service with the new home location
response = set_home_service(request)

# Check if the service call was successful
if response.success:
    rospy.loginfo('New home location set successfully')
else:
    rospy.logerr('Failed to set new home location')
