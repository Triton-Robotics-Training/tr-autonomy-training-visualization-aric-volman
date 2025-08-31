# constants
uint8 ERR = 0
uint8 RED = 1
uint8 BLUE = 2
uint8 HERO = 3
uint8 INFANTRY = 4
uint8 SENTRY = 5

tr_messages/Point robot_center
uint8 color
uint8 size
float32 distance
vision_msgs/BoundingBox2D[] panels
tr_messages/LightBarPair[] lightbar_pairs
tr_messages/Pose pose