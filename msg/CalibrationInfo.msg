std_msgs/Header header

int8 TOBII_CALIBRATION_POINT_STATUS_FAILED_OR_INVALID = 0
int8 TOBII_CALIBRATION_POINT_STATUS_VALID_BUT_NOT_USED_IN_CALIBRATION = 1
int8 TOBII_CALIBRATION_POINT_STATUS_VALID_AND_USED_IN_CALIBRATION = 2
int8 TOBII_CALIBRATION_POINT_STATUS_UNKNOWN = -1

uint32 calibration_id
geometry_msgs/Point[] screen_point
int8[] left_status
geometry_msgs/Point[] left_points
int8[] right_status
geometry_msgs/Point[] right_points
