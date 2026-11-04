# Header stamp is bridge receipt/timeout time, not sensor acquisition time.
std_msgs/Header header

uint8 SAMPLE_VALID=1
uint8 ADC_RAIL_LOW=2
uint8 ADC_RAIL_HIGH=4
uint8 OUT_OF_RANGE=8
uint8 LINK_STALE=128

uint16 raw_adc
uint16 sequence
uint8 status
# Electrical plausibility only: does not prove an analog sensor is connected.
bool valid
# True only when the operator explicitly enables mounting calibration.
bool calibrated
float32 sensor_voltage
# Sensor shaft angle, 0..2*pi radians; NaN for invalid data.
float32 absolute_angle_rad
# Centered steering angle (positive left), radians; NaN unless valid+calibrated.
float32 angle_rad
