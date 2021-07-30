const { readFile } = require('fs')


// struct Record {
  // uint32_t time;
  // uint8_t proximity;
  // uint8_t steps;
  // uint8_t _[2]; // make padding explicit
  // uint16_t r, g, b, c;
  // int32_t mic;
  // float temperature, pressure, altitude;
  // float magnetic_x, magnetic_y, magnetic_z;
  // float accel_x, accel_y, accel_z;
  // float gyro_x, gyro_y, gyro_z;
  // float humidity;
  // float batt;
// } record;

/**
 * 
 * @param {Buffer} chunk 
 */
const parseRecord = chunk => {
  let p = 0

  const time        = chunk.readUInt32LE(p);  p += 4
  const proximity   = chunk.readUInt8(p);     p += 1
  const steps       = chunk.readUInt8(p);     p += 1
  p += 2; // skip padding of two bytes
  const r           = chunk.readUInt16LE(p);  p += 2
  const g           = chunk.readUInt16LE(p);  p += 2
  const b           = chunk.readUInt16LE(p);  p += 2
  const c           = chunk.readUInt16LE(p);  p += 2
  const mic         = chunk.readInt32LE(p);   p += 4
  const temperature = chunk.readFloatLE(p);   p += 4
  const pressure    = chunk.readFloatLE(p);   p += 4
  const altitude    = chunk.readFloatLE(p);   p += 4
  const magnetic_x  = chunk.readFloatLE(p);   p += 4
  const magnetic_y  = chunk.readFloatLE(p);   p += 4
  const magnetic_z  = chunk.readFloatLE(p);   p += 4
  const accel_x     = chunk.readFloatLE(p);   p += 4
  const accel_y     = chunk.readFloatLE(p);   p += 4
  const accel_z     = chunk.readFloatLE(p);   p += 4
  const gyro_x      = chunk.readFloatLE(p);   p += 4
  const gyro_y      = chunk.readFloatLE(p);   p += 4
  const gyro_z      = chunk.readFloatLE(p);   p += 4
  const humidity    = chunk.readFloatLE(p);   p += 4
  const batt        = chunk.readFloatLE(p);   p += 4
  return {
    time,
    proximity,
    r,
    g,
    b,
    c,
    temperature,
    pressure,
    altitude,
    magnetic_x,
    magnetic_y,
    magnetic_z,
    accel_x,
    accel_y,
    accel_z,
    gyro_x,
    gyro_y,
    gyro_z,
    humidity,
    mic,
    batt,
    steps,
  }
}

// readFile('/Volumes/WALK/BINTEST.DAT', (err, data) => {
readFile(process.argv[2], (err, data) => {
    if (err) {
    console.log(err)
    return
  }

  console.log(data.length, data.length % 76)
  const entries = Math.floor(data.length / 76)
  console.log(`File has ${entries} entries.`)
  const chunks = data.reduce((chunks, _, index) => {
    if (index % 76 == 0) {
      const buff = Buffer.alloc(76)
      data.copy(buff, 0, index, index + 76)
      chunks.push(parseRecord(buff))
      console.log(parseRecord(buff))
    }
    return chunks
  }, [])

})