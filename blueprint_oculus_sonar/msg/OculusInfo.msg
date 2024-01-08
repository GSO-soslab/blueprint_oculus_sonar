### IO timestamp or sonar timestamp
Header header

float32[] tvg

## Fields from OculusMessageHeader
uint16 src_device_id    # The device id of the source
uint16 dst_device_id    # The device id of the destination
uint16 msg_id           # Message identifier
uint16 msg_version
uint32 payload_size     # The size of the message payload (header not included)
#uint16 header_spare2

## Fields from OculusSimpleFireMessage / OculusSimpleFireMessage2
uint8 master_mode
uint8 ping_rate          # the ping rate
uint8 network_speed      # The max network speed in Mbs , set to 0x00 or 0xff to use link speed
uint8 gamma_correction   # The gamma correction - 255 is equal to a gamma correction of 1.0
uint8 flags
float64 range            # The range  -- could be meters or percent?

float64 gain_percent     # The percentage gain
float64 speed_of_sound   # The speed of sound - set to zero to use internal calculations
float64 salinity         # The salinity to be used with internal speed of sound calculations (ppt)
uint32 ext_flags

## Fields from OculusSimplePingResult / OculusSimplePingResult2
uint32 ping_id  		   # An incrementing number
uint32 status
float64 frequency		   # The acoustic frequency (Hz)
float64 temperature		 # The external temperature (deg C)
float64 pressure			 # The external pressure (bar)
float64 heading			   # The heading (degrees)
float64 pitch			     # The pitch (degrees)
float64 roll			     # The roll (degrees)
float64 speed_of_sound_used  # The actual used speed of sound (m/s)
float64 ping_start_time   # In seconds from sonar powerup (to microsecond resolution)
uint8 data_size           # The size of the individual data entries 
                          # Remember this is the _raw_ value from the Oculus,
                          # which goes 1-2-3, not 1-2-4 (bytes)
float64 range_resolution  # The range in metres corresponding to a single range line
uint16 n_ranges		     # The number of range lines in the image
uint16 n_beams		     # The number of bearings in the image
#uint32 spare0
#uint32 spare1
#uint32 spare2
#uint32 spare3
uint32 image_offset      # The offset in bytes of the image data from the start
uint32 image_size        # The size in bytes of the image data
uint32 message_size      # The total size in bytes of the network message


