# RPLIDAR A1 Scanner

This project provides a simple Python script to connect to a RPLIDAR A1 sensor, retrieve basic information, and read scan data. It is designed to be easily deployed on systems like Ubuntu (e.g., a machine named RSO2) and Raspberry Pi.

## Prerequisites

- Python 3
- RPLIDAR A1 connected to your device (usually via USB)

## Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd rplidar_a1_scan
    ```

2.  **Install the required Python libraries:**
    All dependencies are listed in the `requirements.txt` file. You can install them using pip:
    ```bash
    pip install -r requirements.txt
    ```

## Usage

1.  **Check the RPLIDAR port:**
    Before running the script, you need to identify the correct serial port for your RPLIDAR. On Linux systems (like Ubuntu or Raspberry Pi OS), it is typically `/dev/ttyUSB0`. You can check available ports with the following command:
    ```bash
    ls /dev/tty*
    ```
    If your RPLIDAR is connected to a different port, you will need to update the `PORT_NAME` variable in the `main.py` script.

2.  **Grant port permissions (if needed):**
    You may need to grant your user permission to access the serial port. You can do this by adding your user to the `dialout` group:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    **Important:** You will need to log out and log back in for this change to take effect.

3.  **Run the script:**
    Execute the main script to start scanning:
    ```bash
    python3 main.py
    ```

The script will:
- Connect to the RPLIDAR.
- Print the device information (`get_info()`).
- Print the device health status (`get_health()`).
- Start scanning and print the number of measurements received in each scan for a limited number of cycles.

To stop the script, press `Ctrl+C`. The program will safely stop the LIDAR's motor and disconnect.