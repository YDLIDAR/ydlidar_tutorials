import os
import ydlidar

if __name__ == "__main__":
    ydlidar.os_init();
    laser = ydlidar.CYdLidar();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan()
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
            else :
                print("Failed to get Lidar Data.")
        laser.turnOff();
    laser.disconnecting();
