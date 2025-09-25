import sys
from rqt_gui.main import Main

def main():
    # Entry point for the standalone rqt plugin
    plugin = 'rqt_rplidar_plot.rplidar_plot.RPLidarPlotPlugin'
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

if __name__ == '__main__':
    main()