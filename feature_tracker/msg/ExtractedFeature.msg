int32[] PointID
float64[] keyPoint
int32[] keyPoint_shape
float32[] vecline
int32[] vecline_shape
int32[] lineID
float32[] linedescriptor
int32[] linedescriptor_shape
float32[] pointdescriptor
int32[] pointdescriptor_shape
bool[] valid_points
int32[] valid_points_shape
sensor_msgs/Image image



# ---------------
# -----
# Type of value for key "PointID": <class 'list'> 
# PointID <class 'int'>
# -----
# Type of value for key "keyPoint": <class 'numpy.ndarray'> 
# value.ndim:  2 | value type: float64 | value shape:  (3, 289)
# -----
# Type of value for key "vecline": <class 'numpy.ndarray'> 
# value.ndim:  3 | value type: float32 | value shape:  (629, 2, 2)
# -----
# Type of value for key "lineID": <class 'list'> 
# lineID <class 'int'>
# -----
# Type of value for key "linedescriptor": <class 'torch.Tensor'> 
# value.ndim:  3 | value type: torch.float32 | value shape:  torch.Size([128, 629, 5])
# -----
# Type of value for key "pointdescriptor": <class 'numpy.ndarray'> 
# value.ndim:  2 | value type: float32 | value shape:  (128, 289)
# -----
# Type of value for key "valid_points": <class 'numpy.ndarray'> 
# value.ndim:  2 | value type: bool | value shape:  (629, 5)
# -----
# Type of value for key "image": <class 'numpy.ndarray'> 
# value.ndim:  2 | value type: uint8 | value shape:  (480, 752)
# ---------------
