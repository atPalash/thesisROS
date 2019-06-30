// Auto-generated. Do not edit!

// (in-package thesis_realsense.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripperDataRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.grasp_pose = null;
      this.max_contact_force = null;
      this.max_contact_velocity = null;
      this.max_contact_width = null;
      this.release_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('grasp_pose')) {
        this.grasp_pose = initObj.grasp_pose
      }
      else {
        this.grasp_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('max_contact_force')) {
        this.max_contact_force = initObj.max_contact_force
      }
      else {
        this.max_contact_force = 0.0;
      }
      if (initObj.hasOwnProperty('max_contact_velocity')) {
        this.max_contact_velocity = initObj.max_contact_velocity
      }
      else {
        this.max_contact_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('max_contact_width')) {
        this.max_contact_width = initObj.max_contact_width
      }
      else {
        this.max_contact_width = 0.0;
      }
      if (initObj.hasOwnProperty('release_pose')) {
        this.release_pose = initObj.release_pose
      }
      else {
        this.release_pose = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperDataRequest
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [grasp_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.grasp_pose, buffer, bufferOffset);
    // Serialize message field [max_contact_force]
    bufferOffset = _serializer.float32(obj.max_contact_force, buffer, bufferOffset);
    // Serialize message field [max_contact_velocity]
    bufferOffset = _serializer.float32(obj.max_contact_velocity, buffer, bufferOffset);
    // Serialize message field [max_contact_width]
    bufferOffset = _serializer.float32(obj.max_contact_width, buffer, bufferOffset);
    // Serialize message field [release_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.release_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperDataRequest
    let len;
    let data = new GripperDataRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [grasp_pose]
    data.grasp_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_contact_force]
    data.max_contact_force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_contact_velocity]
    data.max_contact_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_contact_width]
    data.max_contact_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [release_pose]
    data.release_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.grasp_pose);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.release_pose);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'thesis_realsense/GripperDataRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd2d7cc6032573dfb33b714a9555f6f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string id
    
    
    
    
    
    geometry_msgs/PoseStamped grasp_pose
    
    
    float32 max_contact_force
    
    
    float32 max_contact_velocity
    
    
    float32 max_contact_width
    
    
    
    
    
    geometry_msgs/PoseStamped release_pose
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperDataRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.grasp_pose !== undefined) {
      resolved.grasp_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.grasp_pose)
    }
    else {
      resolved.grasp_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.max_contact_force !== undefined) {
      resolved.max_contact_force = msg.max_contact_force;
    }
    else {
      resolved.max_contact_force = 0.0
    }

    if (msg.max_contact_velocity !== undefined) {
      resolved.max_contact_velocity = msg.max_contact_velocity;
    }
    else {
      resolved.max_contact_velocity = 0.0
    }

    if (msg.max_contact_width !== undefined) {
      resolved.max_contact_width = msg.max_contact_width;
    }
    else {
      resolved.max_contact_width = 0.0
    }

    if (msg.release_pose !== undefined) {
      resolved.release_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.release_pose)
    }
    else {
      resolved.release_pose = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

class GripperDataResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasp_result = null;
    }
    else {
      if (initObj.hasOwnProperty('grasp_result')) {
        this.grasp_result = initObj.grasp_result
      }
      else {
        this.grasp_result = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperDataResponse
    // Serialize message field [grasp_result]
    bufferOffset = _serializer.bool(obj.grasp_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperDataResponse
    let len;
    let data = new GripperDataResponse(null);
    // Deserialize message field [grasp_result]
    data.grasp_result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'thesis_realsense/GripperDataResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26f477d83c816f35d1c952351d253e10';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool grasp_result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperDataResponse(null);
    if (msg.grasp_result !== undefined) {
      resolved.grasp_result = msg.grasp_result;
    }
    else {
      resolved.grasp_result = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GripperDataRequest,
  Response: GripperDataResponse,
  md5sum() { return 'd34785af14fea5ec34d952bf17a99f4a'; },
  datatype() { return 'thesis_realsense/GripperData'; }
};
