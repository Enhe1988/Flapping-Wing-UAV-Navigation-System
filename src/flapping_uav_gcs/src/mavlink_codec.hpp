#ifndef FLAPPING_UAV_GCS__MAVLINK_CODEC_HPP_
#define FLAPPING_UAV_GCS__MAVLINK_CODEC_HPP_

#define MAVLINK_USE_MESSAGE_INFO
#include <common/mavlink.h>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace flapping_uav_gcs
{
inline std::string json_string(const char * text, size_t length)
{
  std::ostringstream output;
  output << '"';
  for (size_t i = 0; i < length; ++i) {
    const auto value = static_cast<unsigned char>(text[i]);
    if (value == 0) {break;}
    if (value == '"' || value == '\\') {
      output << '\\' << static_cast<char>(value);
    } else if (value >= 32 && value < 127) {output << static_cast<char>(value);} else {
      output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') <<
        static_cast<unsigned>(value) << std::dec;
    }
  }
  output << '"'; return output.str();
}

inline void json_float(std::ostream & output, double value)
{
  if (std::isfinite(value)) {output << value;} else if (std::isnan(value)) {
    output << "\"nan\"";
  } else {
    output << (value > 0 ? "\"inf\"" : "\"-inf\"");
  }
}

inline unsigned field_width(mavlink_message_type_t type)
{
  switch (type) {
    case MAVLINK_TYPE_CHAR: case MAVLINK_TYPE_UINT8_T: case MAVLINK_TYPE_INT8_T: return 1;
    case MAVLINK_TYPE_UINT16_T: case MAVLINK_TYPE_INT16_T: return 2;
    case MAVLINK_TYPE_UINT32_T: case MAVLINK_TYPE_INT32_T: case MAVLINK_TYPE_FLOAT: return 4;
    default: return 8;
  }
}

inline void json_scalar(
  std::ostream & output, const mavlink_message_t & message,
  mavlink_message_type_t type, unsigned offset)
{
  switch (type) {
    case MAVLINK_TYPE_CHAR: case MAVLINK_TYPE_UINT8_T:
      output << static_cast<unsigned>(_MAV_RETURN_uint8_t(&message, offset)); break;
    case MAVLINK_TYPE_INT8_T:
      output << static_cast<int>(_MAV_RETURN_int8_t(&message, offset)); break;
    case MAVLINK_TYPE_UINT16_T: output << _MAV_RETURN_uint16_t(&message, offset); break;
    case MAVLINK_TYPE_INT16_T: output << _MAV_RETURN_int16_t(&message, offset); break;
    case MAVLINK_TYPE_UINT32_T: output << _MAV_RETURN_uint32_t(&message, offset); break;
    case MAVLINK_TYPE_INT32_T: output << _MAV_RETURN_int32_t(&message, offset); break;
    case MAVLINK_TYPE_UINT64_T: output << _MAV_RETURN_uint64_t(&message, offset); break;
    case MAVLINK_TYPE_INT64_T: output << _MAV_RETURN_int64_t(&message, offset); break;
    case MAVLINK_TYPE_FLOAT: json_float(output, _MAV_RETURN_float(&message, offset)); break;
    case MAVLINK_TYPE_DOUBLE: json_float(output, _MAV_RETURN_double(&message, offset)); break;
    default: output << "null";
  }
}

inline std::string message_json(const mavlink_message_t & input)
{
  const auto * info = mavlink_get_message_info_by_id(input.msgid);
  if (!info) {return "{}";}
  // MAVLink 2 truncates trailing zero bytes; never expose stale parser memory.
  mavlink_message_t message{};
  message.msgid = input.msgid; message.len = input.len;
  std::memcpy(_MAV_PAYLOAD_NON_CONST(&message), _MAV_PAYLOAD(&input), input.len);
  std::ostringstream output;
  output.imbue(std::locale::classic()); output << std::setprecision(17) << '{';
  for (unsigned i = 0; i < info->num_fields; ++i) {
    const auto & field = info->fields[i];
    if (i) {output << ',';}
    output << json_string(field.name, std::strlen(field.name)) << ':';
    if (field.type == MAVLINK_TYPE_CHAR && field.array_length) {
      output << json_string(_MAV_PAYLOAD(&message) + field.wire_offset, field.array_length);
    } else if (field.array_length) {
      output << '[';
      for (unsigned n = 0; n < field.array_length; ++n) {
        if (n) {output << ',';}
        json_scalar(output, message, field.type, field.wire_offset + n * field_width(field.type));
      }
      output << ']';
    } else {json_scalar(output, message, field.type, field.wire_offset);}
  }
  output << '}'; return output.str();
}
}  // namespace flapping_uav_gcs
#endif  // FLAPPING_UAV_GCS__MAVLINK_CODEC_HPP_
