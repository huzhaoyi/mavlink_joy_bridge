# Raw RC/ADC channel data from MAVLink (PWM us or equivalent)
std_msgs/Header header
uint8 source_type     # 1=PWM_US 2=MANUAL_CONTROL 3=SEALIEN_RC
uint8 link_ok         # 1=recent frame received
int8 mode             # -1/0/1 when known, 127=unused
uint16[] channels     # up to 18 entries, typically 1000~2000 us
