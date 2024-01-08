# Directly inspired by WHOI's ds_core_msgs/RawData.msg
# https://bitbucket.org/whoidsl/ds_msgs/src/master/ds_core_msgs/msg/RawData.msg
#
# Used for logging raw bytes sent to/from hardware.

std_msgs/Header header

int8 DATA_OUT=0  # Data published by the driver
int8 DATA_IN=1  # Data received by the driver

int8 direction

uint8[] data
