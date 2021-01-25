// Auto-generated. Do not edit!

// (in-package lejun_jiang_roslab.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class scan_range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.closest_point = null;
      this.farthest_point = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('closest_point')) {
        this.closest_point = initObj.closest_point
      }
      else {
        this.closest_point = 0.0;
      }
      if (initObj.hasOwnProperty('farthest_point')) {
        this.farthest_point = initObj.farthest_point
      }
      else {
        this.farthest_point = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scan_range
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [closest_point]
    bufferOffset = _serializer.float64(obj.closest_point, buffer, bufferOffset);
    // Serialize message field [farthest_point]
    bufferOffset = _serializer.float64(obj.farthest_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scan_range
    let len;
    let data = new scan_range(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [closest_point]
    data.closest_point = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [farthest_point]
    data.farthest_point = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lejun_jiang_roslab/scan_range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '492e98f4ad5413e40d76ac2fceece9f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 closest_point
    float64 farthest_point
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
    const resolved = new scan_range(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.closest_point !== undefined) {
      resolved.closest_point = msg.closest_point;
    }
    else {
      resolved.closest_point = 0.0
    }

    if (msg.farthest_point !== undefined) {
      resolved.farthest_point = msg.farthest_point;
    }
    else {
      resolved.farthest_point = 0.0
    }

    return resolved;
    }
};

module.exports = scan_range;
