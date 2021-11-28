/**
 * bin2cvs - Custom helper program to convert binary Record data to CSV
 *
 * Copyright (C) 2021  jens alexander ewald <jens@poetic.systems>
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * ------
 *
 * This project has received funding from the European Union’s Horizon 2020
 * research and innovation programme under the Marie Skłodowska-Curie grant
 * agreement No. 813508.
 */


const { readFile } = require('fs')

process.env.TZ = 'Europe/Berlin'

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

const toDate = time => {
  const utc = new Date(time*1000)
  const date = new Date(utc.getTime()) // correct the time zone, quick hack, needs improvement
  date.setHours(utc.getUTCHours())
  return date
}

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
    time: toDate(time),
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

  const csv = chunks.slice(chunks.length - 3).map(c => Object.values(c).join(',')).join('\n')
  console.log(csv)
  console.log(new Date() +"")
})
