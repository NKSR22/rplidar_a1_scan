#!/usr/bin/env python3
from rplidar import RPLidar
import sys

PORT_NAME = '/dev/ttyUSB0'

def run():
    '''Main function'''
    try:
        lidar = RPLidar(PORT_NAME)

        print('Connecting to RPLIDAR...')
        info = lidar.get_info()
        print(info)

        health = lidar.get_health()
        print(health)

        for i, scan in enumerate(lidar.iter_scans()):
            print('%d: Got %d measurements' % (i, len(scan)))
            if i > 10:
                break

    except KeyboardInterrupt:
        print('Stopping.')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'lidar' in locals() and lidar.motor_running:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()

if __name__ == '__main__':
    run()