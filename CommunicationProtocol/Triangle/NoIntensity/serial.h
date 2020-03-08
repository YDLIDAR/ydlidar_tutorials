#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <limits.h>

/*!
* Enumeration defines the possible bytesizes for the serial port.
*/
typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

/*!
* Enumeration defines the possible parity types for the serial port.
*/
typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

/*!
* Enumeration defines the possible stopbit types for the serial port.
*/
typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

/*!
* Enumeration defines the possible flowcontrol types for the serial port.
*/
typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

/*!
* Structure for setting the timeout of the serial port, times are
* in milliseconds.
*
* In order to disable the interbyte timeout, set it to Timeout::max().
*/
struct Timeout {
#ifdef max
# undef max
#endif
  static unsigned int max() {
    return UINT_MAX;
  }
  /*!
  * Convenience function to generate Timeout structs using a
  * single absolute timeout.
  *
  * \param timeout A long that defines the time in milliseconds until a
  * timeout occurs after a call to read or write is made.
  *
  * \return Timeout struct that represents this simple timeout provided.
  */
  static Timeout simpleTimeout(unsigned int timeout) {
    return Timeout(max(), timeout, 0, timeout, 0);
  }

  /*! Number of milliseconds between bytes received to timeout on. */
  unsigned int inter_byte_timeout;
  /*! A constant number of milliseconds to wait after calling read. */
  unsigned int read_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
  *  calling read.
  */
  unsigned int read_timeout_multiplier;
  /*! A constant number of milliseconds to wait after calling write. */
  unsigned int write_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
  *  calling write.
  */
  unsigned int write_timeout_multiplier;

  explicit Timeout(unsigned int inter_byte_timeout_ = 0,
                   unsigned int read_timeout_constant_ = 0,
                   unsigned int read_timeout_multiplier_ = 0,
                   unsigned int write_timeout_constant_ = 0,
                   unsigned int write_timeout_multiplier_ = 0)
    : inter_byte_timeout(inter_byte_timeout_),
      read_timeout_constant(read_timeout_constant_),
      read_timeout_multiplier(read_timeout_multiplier_),
      write_timeout_constant(write_timeout_constant_),
      write_timeout_multiplier(write_timeout_multiplier_) {
  }
};

#endif // SERIAL_H

