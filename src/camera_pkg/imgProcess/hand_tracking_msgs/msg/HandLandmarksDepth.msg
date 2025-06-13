# HandLandmarksDepth.msg
# 包含手部21个关键点的像素坐标和深度信息

std_msgs/Header header
geometry_msgs/Point[] landmarks # x, y是像素坐标，z是该点的深度（米）