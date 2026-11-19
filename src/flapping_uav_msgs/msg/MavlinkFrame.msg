# Local reception time of a CRC-validated message from the common dialect.
std_msgs/Header header
string source_ip
uint16 source_port
uint8 wire_version
uint8 system_id
uint8 component_id
uint8 sequence
uint32 message_id
string message_name
# Unknown floating-point values are strings "nan", "inf", "-inf" in this JSON.
string fields_json
uint8[] payload
