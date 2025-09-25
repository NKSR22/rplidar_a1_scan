#!/usr/bin/env python3
from rplidar import RPLidar
import sys
import argparse

def run():
    '''Main function'''
    parser = argparse.ArgumentParser(description='RSO2 RPLIDAR Scanner')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port name for the RPLIDAR. Default: /dev/ttyUSB0')
    args = parser.parse_args()

    try:
        lidar = RPLidar(args.port)

        print(f'Connecting to RPLIDAR on port {args.port}...')
        info = lidar.get_info()
        print("RPLIDAR Info:", info)

        health = lidar.get_health()
        print("RPLIDAR Health:", health)

        for i, scan in enumerate(lidar.iter_scans()):
            print('Scan %d: Got %d measurements' % (i, len(scan)))
            if i > 10:
                break

    except KeyboardInterrupt:
        print('Stopping scanner.')
    except Exception as e:
        print(f'Error connecting to RPLIDAR: {e}')
    finally:
        if 'lidar' in locals() and lidar.motor_running:
            print('Stopping motor and disconnecting...')
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()

if __name__ == '__main__':
    run()