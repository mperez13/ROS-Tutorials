// Auto-generated. Do not edit!

// (in-package beginner_tutorials.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddTwoIntsRequest {
  constructor() {
    this.a = 0;
    this.b = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type AddTwoIntsRequest
    // Serialize message field [a]
    bufferInfo = _serializer.int64(obj.a, bufferInfo);
    // Serialize message field [b]
    bufferInfo = _serializer.int64(obj.b, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type AddTwoIntsRequest
    let tmp;
    let len;
    let data = new AddTwoIntsRequest();
    // Deserialize message field [a]
    tmp = _deserializer.int64(buffer);
    data.a = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [b]
    tmp = _deserializer.int64(buffer);
    data.b = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'beginner_tutorials/AddTwoIntsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '36d09b846be0b371c5f190354dd3153e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 a
    int64 b
    
    `;
  }

};

class AddTwoIntsResponse {
  constructor() {
    this.sum = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type AddTwoIntsResponse
    // Serialize message field [sum]
    bufferInfo = _serializer.int64(obj.sum, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type AddTwoIntsResponse
    let tmp;
    let len;
    let data = new AddTwoIntsResponse();
    // Deserialize message field [sum]
    tmp = _deserializer.int64(buffer);
    data.sum = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'beginner_tutorials/AddTwoIntsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b88405221c77b1878a3cbbfff53428d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 sum
    
    
    `;
  }

};

module.exports = {
  Request: AddTwoIntsRequest,
  Response: AddTwoIntsResponse
};
