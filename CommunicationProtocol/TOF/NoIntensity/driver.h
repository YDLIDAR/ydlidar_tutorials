#ifndef __DRIVER_H
#define __DRIVER_H
#include <stdint.h>
#include <stddef.h>

/// one ring max point
#define MAX_POINT_SIZE 3600
/// max buffer size
#define MAX_BUFFER_SIZE 2048
/// 0xAA in packet pos
#define HeaderPos 0
/// Packet Header MSB
#define HEADER_MSB 0xAA
/// Packet Header MSB
#define HEADER_LSB 0x55
/// Packet Header length
#define PackageHeaderSize 10
/// Packet LSN Byte pos
#define LSNPos 3
/// Packet FSA Byte pos
#define FSAPos 4
/// Packet LSA Byte pos
#define LSAPos 6
/// Packet Check code Byte pos
#define CheckCodePos 8
/// Si Byte length
#define SiByteSize 2
/**
 *@brief Lidar serial data cache buffer
 */
typedef struct {
  uint8_t buf[MAX_BUFFER_SIZE];
  uint16_t size;
} lidar_data_t;

/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
  /// lidar angle. unit(rad)
  float angle;
  /// lidar range. unit(m)
  float range;
} LaserPoint;

/**
  * @brief Laser Points Output
  */
typedef struct {
  /// Array of lidar points
  LaserPoint  points[MAX_POINT_SIZE];
  uint16_t    npoints;
} LaserFan;

/**
 * @c SetScanDataCallback response callback function.
 * @param data information of the lidar data, becomes invalid after the function returns.
 */
typedef void (*ScanDataCallback)(const LaserFan *data);

class Lidar {
 public:

  Lidar();

  /**
   * Set the callback of listening Lidar scan Data message. When one ring data message is received from ydlidar, cb
   * is called.
   * @param cb callback for scan data.
   */
  void SetScanDataCallback(ScanDataCallback cb);

  /**
   * @brief lidar_data_parsing
   * @param data_in     serial data
   * @return 1 if parse the complete data, otherwise 0
   */
  uint8_t lidar_data_parsing(uint8_t data_in);

 protected:
  /**
   * @brief exclusive OR
   * @param data one frame lidar data
   * @param size frame size
   * @return check code
   */
  uint16_t XOR16(uint8_t *data, uint16_t size);
  /**
   * @brief decode buffer to points
   * @param buff    lidar buffer
   * @param len     buffer size
   */
  void bufDecode(uint8_t *buff, uint8_t len);
 private:
  uint8_t last_byte;///< last byte
  uint8_t rx_buf[MAX_BUFFER_SIZE]; ///< rx cache buffer
  uint16_t rx_ptr;///< rx length
  float AngleFSA;///< FSA angle
  float AngleLSA;///< LSA angle
  uint8_t LSN;///< LSN
  LaserFan laserData;///< parsing laser data
  LaserFan scanData;///< Parsed
  lidar_data_t lidar_data;/// lidar serial buffer
  ScanDataCallback m_callback;///< callback
};
#endif
