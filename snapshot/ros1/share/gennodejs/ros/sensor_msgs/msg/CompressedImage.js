// Auto-generated. Do not edit!

// (in-package sensor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CompressedImage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.format = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('format')) {
        this.format = initObj.format
      }
      else {
        this.format = '';
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CompressedImage
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [format]
    bufferOffset = _serializer.string(obj.format, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint8(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CompressedImage
    let len;
    let data = new CompressedImage(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [format]
    data.format = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.format);
    length += object.data.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sensor_msgs/CompressedImage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f7a12909da2c9d3332d540a0977563f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message contains a compressed image
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
    
    string format        # Specifies the format of the data
                         # Acceptable values differ by the image transport used:
                         # - compressed_image_transport:
                         #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]
                         #   where:
                         #   - ORIG_PIXFMT is pixel format of the raw image, i.e.
                         #     the content of sensor_msgs/Image/encoding with
                         #     values from include/sensor_msgs/image_encodings.h
                         #   - CODEC is one of [jpeg, png]
                         #   - COMPRESSED_PIXFMT is only appended for color images
                         #     and is the pixel format used by the compression
                         #     algorithm. Valid values for jpeg encoding are:
                         #     [bgr8, rgb8]. Valid values for png encoding are:
                         #     [bgr8, rgb8, bgr16, rgb16].
                         #   If the field is empty or does not correspond to the
                         #   above pattern, the image is treated as bgr8 or mono8
                         #   jpeg image (depending on the number of channels).
                         # - compressed_depth_image_transport:
                         #     ORIG_PIXFMT; compressedDepth CODEC
                         #   where:
                         #   - ORIG_PIXFMT is pixel format of the raw image, i.e.
                         #     the content of sensor_msgs/Image/encoding with
                         #     values from include/sensor_msgs/image_encodings.h
                         #     It is usually one of [16UC1, 32FC1].
                         #   - CODEC is one of [png, rvl]
                         #   If the field is empty or does not correspond to the
                         #   above pattern, the image is treated as png image.
                         # - Other image transports can store whatever values they
                         #   need for successful decoding of the image. Refer to
                         #   documentation of the other transports for details.
    
    uint8[] data         # Compressed image buffer
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CompressedImage(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.format !== undefined) {
      resolved.format = msg.format;
    }
    else {
      resolved.format = ''
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = CompressedImage;
