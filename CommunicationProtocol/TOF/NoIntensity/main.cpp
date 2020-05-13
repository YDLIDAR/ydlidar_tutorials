//
// The MIT License (MIT)
//
// Copyright (c) 2019 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#if defined(_WIN32)
#include "win_serial.h"
#else
#include "unix_serial.h"
#include <unistd.h>
#endif
#include "driver.h"

bool ret = false;
Serial serial;


void SigHandler(int sig) {
  if (ret) {
    uint8_t buf[2] = {0xa5, 0x65};
    serial.write(buf, 2);
  }

  serial.close();
  ret = false;
}

void onScanDataCallback(const LaserFan *data) {
  printf("size, %d\n", data->npoints);
  fflush(stdout);
}


int main(int argc, const char *argv[]) {
  Lidar lidar;
  Timeout timeout;
  timeout = timeout.simpleTimeout(2000);
  serial.setTimeout(timeout);
#if defined(_WIN32)
  ret = serial.open("COM7", 512000);
#else
  ret = serial.open("/dev/ttyUSB0", 512000);
#endif
  serial.flushInput();
  lidar.SetScanDataCallback(onScanDataCallback);
  signal(SIGINT, &SigHandler);//Signal Interrupt Ctrl+C
  signal(SIGTERM, &SigHandler);//Signal Terminate


  if (ret) {
    uint8_t buf[2] = {0xa5, 0x60};
    serial.write(buf, 2);
  }

  while (ret) {
    if (serial.available()) {
      while (serial.available()) {
        uint8_t buf;

        if (serial.read(&buf, 1)) {
          if (lidar.lidar_data_parsing(buf)) {
          }
        }
      }
    } else {
#if defined(_WIN32)
      Sleep(10);
#else
      usleep(1000 * 10);
#endif
    }
  }

  if (ret) {
    uint8_t buf[2] = {0xa5, 0x65};
    serial.write(buf, 2);
  }

  serial.close();

  return 0;

}
