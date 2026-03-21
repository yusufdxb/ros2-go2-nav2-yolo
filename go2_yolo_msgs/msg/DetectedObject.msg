# go2_yolo_msgs/msg/DetectedObject.msg
string class_name
int32 class_id
float32 confidence
geometry_msgs/Point position   # 3-D position in camera_color_optical_frame (metres)
int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2
int32 track_id                 # BoT-SORT persistent track ID; -1 = no tracker active
float32 lock_score             # Cosine similarity to locked target gallery; 0.0 = not locked
