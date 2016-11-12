// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Num {
  constructor() {
    this.num = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Num
    // Serialize message field [num]
    bufferInfo = _serializer.int64(obj.num, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Num
    let tmp;
    let len;
    let data = new Num();
    // Deserialize message field [num]
    tmp = _deserializer.int64(buffer);
    data.num = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/Num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57d3c40ec3ac3754af76a83e6e73127a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 num
    
    `;
  }

};

module.exports = Num;
