// Auto-generated. Do not edit!

// (in-package simulation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class VehicleControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Throttle = null;
      this.Steering = null;
      this.Brake = null;
      this.Reserved = null;
    }
    else {
      if (initObj.hasOwnProperty('Throttle')) {
        this.Throttle = initObj.Throttle
      }
      else {
        this.Throttle = 0.0;
      }
      if (initObj.hasOwnProperty('Steering')) {
        this.Steering = initObj.Steering
      }
      else {
        this.Steering = 0.0;
      }
      if (initObj.hasOwnProperty('Brake')) {
        this.Brake = initObj.Brake
      }
      else {
        this.Brake = 0.0;
      }
      if (initObj.hasOwnProperty('Reserved')) {
        this.Reserved = initObj.Reserved
      }
      else {
        this.Reserved = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleControl
    // Serialize message field [Throttle]
    bufferOffset = _serializer.float32(obj.Throttle, buffer, bufferOffset);
    // Serialize message field [Steering]
    bufferOffset = _serializer.float32(obj.Steering, buffer, bufferOffset);
    // Serialize message field [Brake]
    bufferOffset = _serializer.float32(obj.Brake, buffer, bufferOffset);
    // Serialize message field [Reserved]
    bufferOffset = _serializer.float32(obj.Reserved, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleControl
    let len;
    let data = new VehicleControl(null);
    // Deserialize message field [Throttle]
    data.Throttle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Steering]
    data.Steering = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Brake]
    data.Brake = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Reserved]
    data.Reserved = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'simulation/VehicleControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d66040a38beac1d32107b24d8aabeae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 Throttle
    float32 Steering
    float32 Brake
    float32 Reserved
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehicleControl(null);
    if (msg.Throttle !== undefined) {
      resolved.Throttle = msg.Throttle;
    }
    else {
      resolved.Throttle = 0.0
    }

    if (msg.Steering !== undefined) {
      resolved.Steering = msg.Steering;
    }
    else {
      resolved.Steering = 0.0
    }

    if (msg.Brake !== undefined) {
      resolved.Brake = msg.Brake;
    }
    else {
      resolved.Brake = 0.0
    }

    if (msg.Reserved !== undefined) {
      resolved.Reserved = msg.Reserved;
    }
    else {
      resolved.Reserved = 0.0
    }

    return resolved;
    }
};

module.exports = VehicleControl;
