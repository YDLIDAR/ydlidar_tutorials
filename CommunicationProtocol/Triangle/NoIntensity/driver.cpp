#include "driver.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>

Lidar::Lidar(void) {
  memset(&laserData, 0, sizeof(LaserFan));
  memset(&scanData, 0, sizeof(LaserFan));
  memset(&lidar_data, 0, sizeof(lidar_data_t));
  memset(rx_buf, 0, MAX_BUFFER_SIZE);
  last_byte = 0x00;
  rx_ptr = 0;
  AngleFSA = 0;
  AngleLSA = 0;
  LSN = 0;
  m_callback = NULL;
}

/**
 * @brief Lidar::SetScanDataCallback
 * @param cb
 */
void Lidar::SetScanDataCallback(ScanDataCallback cb) {
  m_callback = cb;
}

//exclusive OR
uint16_t Lidar::XOR16(uint8_t *data, uint16_t size) {
  uint16_t check_code = 0x00;
  uint16_t len = size / 2;
  uint16_t *data_ptr = (uint16_t *)data;
  uint16_t i = 0;

  if (size % SiByteSize == 1) {
    return 0xffff;
  }

  for (i = 0; i < (PackageHeaderSize - 2) / 2; i++) {
    check_code ^= *data_ptr++;
  }

  data_ptr++;

  for (i = 0; i < len - PackageHeaderSize / 2; i++) {
    check_code ^= *data_ptr++;
  }

  //switch high and low
  return check_code >> 8 | check_code << 8;
}

//all data will catch from serial and send in here
uint8_t Lidar::lidar_data_parsing(uint8_t data) {
  uint8_t ret = 0;
  //plus the data to this buff
  rx_buf[rx_ptr++] = data;

  //
  if (rx_ptr > MAX_BUFFER_SIZE - 2) {
    rx_ptr = 0;
  }

  //we check if we catch the next package.
  //we process the data we catch if the package is already finish.
  if (data == HEADER_LSB && last_byte == HEADER_MSB) {

    //check the head
    if (rx_ptr > PackageHeaderSize && rx_buf[HeaderPos] == HEADER_MSB &&
        rx_buf[HeaderPos + 1] == HEADER_LSB) {

      //copy buf data
      memcpy(lidar_data.buf, rx_buf, rx_ptr);
      lidar_data.size = rx_ptr;
      //decode the LSN
      LSN = lidar_data.buf[LSNPos];

      //decode the FSA Angle and LSA Angle
      AngleFSA = float((lidar_data.buf[FSAPos] | (lidar_data.buf[FSAPos + 1] << 8)) >>
                       1);
      AngleLSA = float((lidar_data.buf[LSAPos] | (lidar_data.buf[LSAPos + 1] << 8)) >>
                       1);
      //check code
      uint16_t check_code = XOR16(lidar_data.buf, rx_ptr - 2);
      uint16_t check_sum = ((rx_buf[CheckCodePos] << 8) | rx_buf[CheckCodePos + 1]);

      if (check_code != check_sum) {
        if (rx_ptr > LSN * SiByteSize + PackageHeaderSize + 2) {
          check_code = XOR16(lidar_data.buf, LSN * SiByteSize + PackageHeaderSize);
        }
      }

      //check code is true
      if (check_code == check_sum) {
        // zero packet
        if ((lidar_data.buf[2] & 0x01) == 0x01) {//new scan data
          //lock copy
          // first ring npoints is zero.
          if (scanData.npoints) {
            ret = 1;
          }

          scanData.npoints = laserData.npoints;
          memcpy(scanData.points, laserData.points,
                 sizeof(LaserPoint) *laserData.npoints);
          laserData.npoints = 0;

          if (m_callback && ret) {
            m_callback(&scanData);
          }

        }

        //decode
        // normal data packet
        bufDecode(&lidar_data.buf[PackageHeaderSize], rx_ptr - PackageHeaderSize - 2);

      } else {
        //xor error
      }
    }

    //keep header which we were already receive.
    if (rx_ptr >= LSN * SiByteSize + PackageHeaderSize + 2) {
      memset(rx_buf, 0, rx_ptr);
      rx_ptr = 0;
      rx_buf[rx_ptr++] = HEADER_MSB;
      rx_buf[rx_ptr++] = HEADER_LSB;
    }
  }

  last_byte = data;
  return ret;
}

//lidar point decode here.
void Lidar::bufDecode(uint8_t *buff, uint8_t len) {
  LaserPoint point;
  float AngCorrect = 0;
  // one data is 2 byte, data size = len / 2
  int data_size = len / SiByteSize;
  // angle diff
  float AngleDiff = AngleFSA - AngleFSA;
  int i = 0;
  point.angle = 0;
  point.range = 0;

  // angle diff less than zero,
  if (AngleDiff < 0.0) {
    AngleDiff = AngleDiff + 360.0 * 64;
  }

  AngleDiff = AngleDiff / ((data_size - 1) * 1.0);

  for (i = 0; i < len; i += SiByteSize) {
    point.angle = AngleDiff * (i / SiByteSize) + AngleFSA;
    point.range = float(uint16_t(buff[i] | (buff[i + 1] << 8)) / 4.0);

    if (point.range > 0) {
      AngCorrect = (atan(21.8 * (155.3 - point.range) / (155.3 * point.range)) * 180 /
                    3.1415) * 64.0;
    } else {
      AngCorrect = 0;
    }

    point.angle = (point.angle + AngCorrect) / 64.0;

    if (point.angle > 360.0) {// FSA > LSA
      point.angle =  point.angle - 360;
    }

    point.angle =  point.angle * 3.1415 / 180;
    point.range = point.range / 1000.0;
    laserData.points[laserData.npoints++] = point;

    if (laserData.npoints >= MAX_POINT_SIZE) {
      laserData.npoints--;
    }
  }
}

