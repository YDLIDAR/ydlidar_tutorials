#if defined(_WIN32)

#include "win_serial.h"
#include <string>

static inline void set_common_props(DCB *dcb) {
  dcb->fBinary = TRUE;
  dcb->fAbortOnError = FALSE;
  dcb->fNull = FALSE;
  dcb->fErrorChar = FALSE;

  if (dcb->fDtrControl == DTR_CONTROL_HANDSHAKE) {
    dcb->fDtrControl = DTR_CONTROL_DISABLE;
  }

  if (dcb->fRtsControl != RTS_CONTROL_HANDSHAKE) {
    dcb->fRtsControl = RTS_CONTROL_DISABLE;
  }
}

static inline void set_baudrate(DCB *dcb, unsigned long baudrate) {
  // setup baud rate
  switch (baudrate) {
#ifdef CBR_0

    case 0:
      dcb->BaudRate = CBR_0;
      break;
#endif
#ifdef CBR_50

    case 50:
      dcb->BaudRate = CBR_50;
      break;
#endif
#ifdef CBR_75

    case 75:
      dcb->BaudRate = CBR_75;
      break;
#endif
#ifdef CBR_110

    case 110:
      dcb->BaudRate = CBR_110;
      break;
#endif
#ifdef CBR_134

    case 134:
      dcb->BaudRate = CBR_134;
      break;
#endif
#ifdef CBR_150

    case 150:
      dcb->BaudRate = CBR_150;
      break;
#endif
#ifdef CBR_200

    case 200:
      dcb->BaudRate = CBR_200;
      break;
#endif
#ifdef CBR_300

    case 300:
      dcb->BaudRate = CBR_300;
      break;
#endif
#ifdef CBR_600

    case 600:
      dcb->BaudRate = CBR_600;
      break;
#endif
#ifdef CBR_1200

    case 1200:
      dcb->BaudRate = CBR_1200;
      break;
#endif
#ifdef CBR_1800

    case 1800:
      dcb->BaudRate = CBR_1800;
      break;
#endif
#ifdef CBR_2400

    case 2400:
      dcb->BaudRate = CBR_2400;
      break;
#endif
#ifdef CBR_4800

    case 4800:
      dcb->BaudRate = CBR_4800;
      break;
#endif
#ifdef CBR_7200

    case 7200:
      dcb->BaudRate = CBR_7200;
      break;
#endif
#ifdef CBR_9600

    case 9600:
      dcb->BaudRate = CBR_9600;
      break;
#endif
#ifdef CBR_14400

    case 14400:
      dcb->BaudRate = CBR_14400;
      break;
#endif
#ifdef CBR_19200

    case 19200:
      dcb->BaudRate = CBR_19200;
      break;
#endif
#ifdef CBR_28800

    case 28800:
      dcb->BaudRate = CBR_28800;
      break;
#endif
#ifdef CBR_57600

    case 57600:
      dcb->BaudRate = CBR_57600;
      break;
#endif
#ifdef CBR_76800

    case 76800:
      dcb->BaudRate = CBR_76800;
      break;
#endif
#ifdef CBR_38400

    case 38400:
      dcb->BaudRate = CBR_38400;
      break;
#endif
#ifdef CBR_115200

    case 115200:
      dcb->BaudRate = CBR_115200;
      break;
#endif
#ifdef CBR_128000

    case 128000:
      dcb->BaudRate = CBR_128000;
      break;
#endif
#ifdef CBR_153600

    case 153600:
      dcb->BaudRate = CBR_153600;
      break;
#endif
#ifdef CBR_230400

    case 230400:
      dcb->BaudRate = CBR_230400;
      break;
#endif
#ifdef CBR_256000

    case 256000:
      dcb->BaudRate = CBR_256000;
      break;
#endif
#ifdef CBR_460800

    case 460800:
      dcb->BaudRate = CBR_460800;
      break;
#endif
#ifdef CBR_921600

    case 921600:
      dcb->BaudRate = CBR_921600;
      break;
#endif

    default:
      // Try to blindly assign it
      dcb->BaudRate = baudrate;
  }
}

static inline void set_databits(DCB *dcb, bytesize_t bytesize) {
  switch (bytesize) {
    case fivebits:
      dcb->ByteSize = 5;
      break;

    case sixbits:
      dcb->ByteSize = 6;
      break;

    case sevenbits:
      dcb->ByteSize = 7;
      break;

    case eightbits:
      dcb->ByteSize = 8;
      break;

    default:
      dcb->ByteSize = 8;
      break;
  }
}

static inline void set_parity(DCB *dcb, parity_t parity) {
  dcb->fParity = TRUE;

  switch (parity) {
    case parity_none:
      dcb->Parity = NOPARITY;
      dcb->fParity = FALSE;
      break;

    case parity_odd:
      dcb->Parity = ODDPARITY;
      break;

    case parity_even:
      dcb->Parity = EVENPARITY;
      break;

    case parity_mark:
      dcb->Parity = MARKPARITY;
      break;

    case parity_space:
      dcb->Parity = SPACEPARITY;
      break;

    default:
      dcb->Parity = NOPARITY;
      dcb->fParity = FALSE;
      break;
  }
}

static inline void set_stopbits(DCB *dcb, stopbits_t stopbits) {
  switch (stopbits) {
    case stopbits_one:
      dcb->StopBits = ONESTOPBIT;
      break;

    case stopbits_one_point_five:
      dcb->StopBits = ONE5STOPBITS;
      break;

    case stopbits_two:
      dcb->StopBits = TWOSTOPBITS;
      break;

    default:
      dcb->StopBits = ONESTOPBIT;
      break;
  }
}

static inline void set_flowcontrol(DCB *dcb, flowcontrol_t flowcontrol) {
  dcb->fInX = FALSE;
  dcb->fOutX = FALSE;
  dcb->fOutxCtsFlow = FALSE;

  if (dcb->fRtsControl == RTS_CONTROL_HANDSHAKE) {
    dcb->fRtsControl = RTS_CONTROL_DISABLE;
  }

  switch (flowcontrol) {
    case flowcontrol_none:
      break;

    case flowcontrol_software:
      dcb->fInX = TRUE;
      dcb->fOutX = TRUE;
      break;

    case flowcontrol_hardware:
      dcb->fOutxCtsFlow = TRUE;
      dcb->fRtsControl = RTS_CONTROL_HANDSHAKE;
      break;

    default:
      break;
  }
}

inline std::wstring _prefix_port_if_needed(const std::wstring &input) {
  static std::wstring windows_com_port_prefix = L"\\\\.\\";

  if (input.compare(windows_com_port_prefix) != 0) {
    return windows_com_port_prefix + input;
  }

  return input;
}

Serial::Serial()
  : fd_(INVALID_HANDLE_VALUE), is_open_(false),
    parity_(parity_none),
    bytesize_(eightbits), stopbits_(stopbits_one), flowcontrol_(flowcontrol_none) {
  read_mutex = CreateMutex(NULL, false, NULL);
  write_mutex = CreateMutex(NULL, false, NULL);
  memset(&_wait_o, 0, sizeof(_wait_o));
  _wait_o.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
}

Serial::~Serial() {
  this->close();
  CloseHandle(_wait_o.hEvent);
  CloseHandle(read_mutex);
  CloseHandle(write_mutex);
}

bool Serial::open(const char *port, unsigned long baudrate) {
  if (strcmp(port, "") == 0) {
    return false;
  }

  if (is_open_ == true) {
    return true;
  }

  DWORD desiredAccess = 0;
  originalEventMask = 0;

  baudrate_ = baudrate;
  desiredAccess |= GENERIC_READ;
  originalEventMask = EV_RXCHAR | EV_ERR ;
  desiredAccess |= GENERIC_WRITE;
  std::string str = port;
  std::wstring port_ = std::wstring(str.begin(), str.end());
  std::wstring port_with_prefix = _prefix_port_if_needed(port_);
  LPCWSTR lp_port = port_with_prefix.c_str();
  fd_ = CreateFile(lp_port,
                   desiredAccess,
                   0,
                   nullptr,
                   OPEN_EXISTING,
                   FILE_ATTRIBUTE_NORMAL |
                   FILE_FLAG_OVERLAPPED,//FILE_FLAG_OVERLAPPED,//FILE_ATTRIBUTE_NORMAL
                   nullptr);

  if (fd_ == INVALID_HANDLE_VALUE) {
    DWORD errno_ = GetLastError();

    switch (errno_) {
      case ERROR_FILE_NOT_FOUND:
      default:
        return false;

    }
  }



  if (reconfigurePort()) {
    is_open_ = true;
    return true;
  }

  ::CloseHandle(fd_);
  return false;

}

bool Serial::reconfigurePort() {
  if (fd_ == INVALID_HANDLE_VALUE) {
    // Can only operate on a valid file descriptor
    return false;
  }

  if (!SetupComm(fd_, DEFAULT_RX_BUFFER_SIZE, DEFAULT_TX_BUFFER_SIZE)) {
    return false;
  }

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  set_common_props(&dcb);
  set_baudrate(&dcb, baudrate_);
  set_databits(&dcb, bytesize_);
  set_parity(&dcb, parity_);
  set_stopbits(&dcb, stopbits_);
  set_flowcontrol(&dcb, flowcontrol_);

  if (!setDcb(&dcb)) {
    return false;
  }

  if (!::GetCommTimeouts(fd_, &restoredCommTimeouts)) {
    return false;
  }

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / dcb.BaudRate;

  ::ZeroMemory(&currentCommTimeouts, sizeof(currentCommTimeouts));
  DWORD serialBitsPerByte = dcb.ByteSize + 1;
  serialBitsPerByte += (dcb.Parity == NOPARITY) ? 0 : 1;
  serialBitsPerByte += (dcb.StopBits == ONESTOPBIT) ? 1 : 2;


  DWORD msPerByte = (dcb.BaudRate > 0) ? ((1000 * serialBitsPerByte + dcb.BaudRate
                                          - 1) /
                                          dcb.BaudRate) : 1;
  currentCommTimeouts.ReadIntervalTimeout =
    msPerByte; //最小化串联端口数据包连接读取的机会// Minimize chance of concatenating of separate serial port packets on read
  currentCommTimeouts.ReadTotalTimeoutMultiplier =
    0; //当使用大的读取缓冲区时，不允许大的读取超时// Do not allow big read timeout when big read buffer used
  currentCommTimeouts.ReadTotalTimeoutConstant =
    2000; //总读取超时（读取循环的周期）// Total read timeout (period of read loop)
  currentCommTimeouts.WriteTotalTimeoutConstant =
    2000; //写超时的常量部分// Const part of write timeout
  currentCommTimeouts.WriteTotalTimeoutMultiplier =
    msPerByte; //写入超时的变量部分（每个字节）// Variable part of write timeout (per byte)

  byte_time_ns_ = bit_time_ns * msPerByte;

  if (!::SetCommTimeouts(fd_, &currentCommTimeouts)) {
    return false;
  }

  if (!::SetCommMask(fd_, originalEventMask)) {
    return false;
  }

  if (!PurgeComm(fd_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR |
                 PURGE_RXCLEAR)) {
    return false;
  }


  ::ZeroMemory(&communicationOverlapped, sizeof(communicationOverlapped));

  if (!(originalEventMask & EV_RXCHAR) &&
      !::WaitCommEvent(fd_, &triggeredEventMask, &communicationOverlapped)) {
    return false;
  }

  return true;

}

void Serial::close() {
  ResetEvent(_wait_o.hEvent);

  if (is_open_ == true) {
    if (fd_ != INVALID_HANDLE_VALUE) {
      ::CancelIo(fd_);
      int ret;
      ret = CloseHandle(fd_);
    }

    fd_ = INVALID_HANDLE_VALUE;
    is_open_ = false;
  }
}

bool Serial::isOpen() const {
  return is_open_;
}

size_t Serial::available() {
  if (!is_open_) {
    return 0;
  }

  COMSTAT cs;
  DWORD error;

  if (ClearCommError(fd_, &error, &cs) && error > 0) {
    PurgeComm(fd_, PURGE_RXABORT | PURGE_RXCLEAR);
    return 0;
  }

  return static_cast<size_t>(cs.cbInQue);
}

bool Serial::waitReadable(uint32_t /*timeout*/) {
  return false;
}

void Serial::waitByteTimes(size_t /*count*/) {
  return ;
}

int  Serial::waitfordata(size_t data_count, uint32_t timeout,
                         size_t *returned_size) {
  if (!is_open_) {
    return 0;
  }

  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  *returned_size = 0;

  if (is_open_) {
    size_t queue_remaining =  available();

    if (queue_remaining >= data_count) {
      *returned_size = queue_remaining;
      return 0;
    }
  }

  COMSTAT  stat;
  DWORD error;
  DWORD msk, lengths;

  while (is_open_) {
    msk = 0;
    SetCommMask(fd_, EV_RXCHAR | EV_ERR);

    if (!WaitCommEvent(fd_, &msk, &_wait_o)) {
      if (GetLastError() == ERROR_IO_PENDING) {

        if (WaitForSingleObject(_wait_o.hEvent, timeout) == WAIT_TIMEOUT) {
          *returned_size = 0;
          return -1;
        }

        GetOverlappedResult(fd_, &_wait_o, &lengths, TRUE);

        ::ResetEvent(_wait_o.hEvent);
      } else {
        ClearCommError(fd_, &error, &stat);
        *returned_size = stat.cbInQue;
        return -2;
      }
    }

    if (msk & EV_ERR) {
      // FIXME: may cause problem here
      ClearCommError(fd_, &error, &stat);
    }

    if (msk & EV_RXCHAR) {
      ClearCommError(fd_, &error, &stat);

      if (stat.cbInQue >= data_count) {
        *returned_size = stat.cbInQue;
        return 0;
      }
    }
  }

  *returned_size = 0;
  return -2;
}


size_t Serial::read(uint8_t *buf, size_t size) {
  if (!is_open_) {
    return 0;
  }

  DWORD bytes_read;
  ::ZeroMemory(&readCompletionOverlapped, sizeof(readCompletionOverlapped));

  if (!ReadFile(fd_, buf, static_cast<DWORD>(size), &bytes_read,
                &readCompletionOverlapped)) {
    if (GetLastError() == ERROR_IO_PENDING) {
      DWORD dwWait =::WaitForSingleObject(fd_, INFINITE);

      if (dwWait != WAIT_OBJECT_0) {
        if (!GetOverlappedResult(fd_, NULL, &bytes_read, TRUE)) {
          if (GetLastError() != ERROR_IO_INCOMPLETE) {
            return 0;
          }
        }
      }
    }

    return 0;
  }

  return (int)(bytes_read);
}

size_t Serial::write(const uint8_t *data, size_t length) {
  if (is_open_ == false) {
    return 0;
  }

  if (data == NULL || length == 0) {
    return 0;
  }

  DWORD    error;

  if (ClearCommError(fd_, &error, NULL) && error > 0) {
    PurgeComm(fd_, PURGE_TXABORT | PURGE_TXCLEAR);
  }

  DWORD bytes_written;
  ::ZeroMemory(&writeCompletionOverlapped, sizeof(writeCompletionOverlapped));

  if (!::WriteFile(fd_, data, static_cast<DWORD>(length), &bytes_written,
                   &writeCompletionOverlapped)) {
    return 0;
  }

  return (int)(bytes_written);
}

void Serial::setTimeout(Timeout &timeout) {
  timeout_ = timeout;

  if (fd_ == INVALID_HANDLE_VALUE) {
    return;
  }

  COMMTIMEOUTS currentTimeouts;

  if (!::GetCommTimeouts(fd_, &currentTimeouts)) {
    return;
  }

  // Setup timeouts
  ::ZeroMemory(&currentCommTimeouts, sizeof(currentCommTimeouts));
  currentCommTimeouts.ReadIntervalTimeout =
    currentTimeouts.ReadIntervalTimeout;//MAXWORD;
  currentCommTimeouts.ReadTotalTimeoutConstant = timeout_.read_timeout_constant;
  currentCommTimeouts.ReadTotalTimeoutMultiplier =
    timeout_.read_timeout_multiplier;
  currentCommTimeouts.WriteTotalTimeoutConstant = timeout_.write_timeout_constant;
  currentCommTimeouts.WriteTotalTimeoutMultiplier =
    currentTimeouts.WriteTotalTimeoutMultiplier;//timeout_.write_timeout_multiplier;

  if (!SetCommTimeouts(fd_, &currentCommTimeouts)) {
    return;
  }

}

Timeout Serial::getTimeout() const {
  return timeout_;
}


bool Serial::setDcb(DCB *dcb) {
  if (!::SetCommState(fd_, dcb)) {
    return false;
  }

  return true;
}

bool Serial::getDcb(DCB *dcb) {
  ::ZeroMemory(dcb, sizeof(DCB));
  dcb->DCBlength = sizeof(DCB);

  if (!::GetCommState(fd_, dcb)) {
    return false;
  }

  return true;
}


bool Serial::setBaudrate(unsigned long baudrate) {
  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  baudrate_ = baudrate;
  set_baudrate(&dcb, baudrate);

  return setDcb(&dcb);
}

unsigned long Serial::getBaudrate() const {
  return baudrate_;
}

bool Serial::setBytesize(bytesize_t bytesize) {

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  bytesize_ = bytesize;
  set_databits(&dcb, bytesize);

  return setDcb(&dcb);
}

bytesize_t Serial::getBytesize() const {
  return bytesize_;
}

bool Serial::setParity(parity_t parity) {

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  parity_ = parity;
  set_parity(&dcb, parity);

  return setDcb(&dcb);
}

parity_t Serial::getParity() const {
  return parity_;
}

bool Serial::setStopbits(stopbits_t stopbits) {

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  stopbits_ = stopbits;
  set_stopbits(&dcb, stopbits);

  return setDcb(&dcb);
}

stopbits_t Serial::getStopbits() const {
  return stopbits_;
}

bool Serial::setFlowcontrol(flowcontrol_t flowcontrol) {

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  flowcontrol_ = flowcontrol;
  set_flowcontrol(&dcb, flowcontrol);

  return setDcb(&dcb);
}

flowcontrol_t Serial::getFlowcontrol() const {
  return flowcontrol_;
}

void Serial::flush() {
  if (is_open_ == false) {
    return;
  }

  //FlushFileBuffers (fd_);
  PurgeComm(fd_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
}

void Serial::flushInput() {
  return;
}

void Serial::flushOutput() {
  return;
}

void Serial::sendBreak(int duration) {
  if (!setBreak(true)) {
    return ;
  }

  ::Sleep(duration);

  if (!setBreak(false)) {
    return ;
  }

}

bool Serial::setBreak(bool level) {
  if (is_open_ == false) {
    return false;
  }

  if (level) {
    //EscapeCommFunction (fd_, SETBREAK);
    //::SetCommBreak(fd_);
    if (::SetCommBreak(fd_) == FALSE) {
      return false;
    }
  } else {
    //::ClearCommBreak(fd_);
    //EscapeCommFunction (fd_, CLRBREAK);
    if (::ClearCommBreak(fd_) == FALSE) {
      return false;
    }
  }

  return true;
}

bool Serial::setRTS(bool level) {
  if (is_open_ == false) {
    return false;
  }

  if (level) {
    //EscapeCommFunction (fd_, SETRTS);
    if (EscapeCommFunction(fd_, SETRTS) == FALSE) {
      return false;
    }
  } else {
    //EscapeCommFunction (fd_, CLRRTS);
    if (EscapeCommFunction(fd_, CLRRTS) == FALSE) {
      return false;
    }
  }

  return true;
}

bool Serial::setDTR(bool level) {
  if (is_open_ == false) {
    return false;
  }

  if (level) {
    //EscapeCommFunction (fd_, SETDTR);
    if (EscapeCommFunction(fd_, SETDTR) == FALSE) {
      return false;
    }
  } else {
    //EscapeCommFunction (fd_, CLRDTR);
    if (EscapeCommFunction(fd_, CLRDTR) == FALSE) {
      return false;
    }
  }

  DCB dcb;

  if (!getDcb(&dcb)) {
    return false;
  }

  dcb.fDtrControl = level ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
  return setDcb(&dcb);
}

bool Serial::waitForChange() {
  if (is_open_ == false) {
    return false;
  }

  DWORD dwCommEvent;

  if (!SetCommMask(fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD)) {
    // Error setting communications mask
    return false;
  }

  if (!WaitCommEvent(fd_, &dwCommEvent, NULL)) {
    // An error occurred waiting for the event.
    return false;
  } else {
    // Event has occurred.
    return true;
  }
}

bool Serial::getCTS() {
  if (is_open_ == false) {
    return false;
  }

  DWORD dwModemStatus;

  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    return false;
  }

  return (MS_CTS_ON & dwModemStatus) != 0;
}

bool Serial::getDSR() {
  if (is_open_ == false) {
    return false;
  }

  DWORD dwModemStatus;

  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    return false;
  }

  return (MS_DSR_ON & dwModemStatus) != 0;
}

bool Serial::getRI() {
  if (is_open_ == false) {
    return false;
  }

  DWORD dwModemStatus;

  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    return false;
  }

  return (MS_RING_ON & dwModemStatus) != 0;
}

bool Serial::getCD() {
  if (is_open_ == false) {
    return false;
  }

  DWORD dwModemStatus;

  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    // Error in GetCommModemStatus;
    return false;
  }

  return (MS_RLSD_ON & dwModemStatus) != 0;
}

uint32_t Serial::getByteTime() {
  return byte_time_ns_;
}


int Serial::readLock() {
  if (WaitForSingleObject(read_mutex, INFINITE) != WAIT_OBJECT_0) {
    return 1;
  }

  return 0;
}

int Serial::readUnlock() {
  if (!ReleaseMutex(read_mutex)) {
    return 1;
  }

  return 0;
}

int Serial::writeLock() {
  if (WaitForSingleObject(write_mutex, INFINITE) != WAIT_OBJECT_0) {
    return 1;
  }

  return 0;
}

int Serial::writeUnlock() {
  if (!ReleaseMutex(write_mutex)) {
    return 1;
  }

  return 0;
}
#endif // #if defined(_WIN32)

