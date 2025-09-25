# RSO2 RPLIDAR Scanner / โปรแกรมสแกนเนอร์สำหรับ RPLIDAR

This project provides a simple Python script to connect to a RPLIDAR A1 sensor, retrieve basic information, and read scan data. It is designed to be easily deployed on systems like Ubuntu (e.g., a machine named RSO2) and Raspberry Pi.

โปรเจคนี้คือสคริปต์ Python สำหรับเชื่อมต่อกับเซ็นเซอร์ RPLIDAR A1 เพื่อดึงข้อมูลพื้นฐานและอ่านค่าการสแกน ถูกออกแบบมาให้สามารถนำไปติดตั้งและใช้งานบนระบบปฏิบัติการ Ubuntu (เช่น เครื่อง RSO2) และ Raspberry Pi ได้อย่างง่ายดาย

## Prerequisites / สิ่งที่ต้องมี

- Python 3
- RPLIDAR A1 connected to your device (usually via USB) / เซ็นเซอร์ RPLIDAR A1 ที่เชื่อมต่อกับอุปกรณ์ของคุณ (โดยทั่วไปผ่าน USB)

## Installation / การติดตั้ง

1.  **Clone the repository (or download the files):** / **คัดลอกโปรเจค (หรือดาวน์โหลดไฟล์):**
    ```bash
    git clone <repository_url>
    cd rplidar_a1_scan
    ```

2.  **Install the required Python libraries:** / **ติดตั้งไลบรารีที่จำเป็น:**
    All dependencies are listed in the `requirements.txt` file. You can install them using pip. / ไลบรารีที่จำเป็นทั้งหมดถูกระบุไว้ในไฟล์ `requirements.txt` สามารถติดตั้งได้ผ่านคำสั่ง pip
    ```bash
    pip install -r requirements.txt
    ```

## Usage / วิธีการใช้งาน

1.  **Check the RPLIDAR port:** / **ตรวจสอบพอร์ทของ RPLIDAR:**
    Before running the script, you need to identify the correct serial port for your RPLIDAR. On Linux systems (like Ubuntu or Raspberry Pi OS), it is typically `/dev/ttyUSB0`. You can check available ports with the following command:
    ก่อนรันสคริปต์ คุณจำเป็นต้องระบุพอร์ทที่ถูกต้องสำหรับ RPLIDAR ของคุณ บนระบบ Linux (เช่น Ubuntu หรือ Raspberry Pi OS) โดยปกติจะเป็น `/dev/ttyUSB0` คุณสามารถตรวจสอบพอร์ทที่ใช้งานได้ด้วยคำสั่ง:
    ```bash
    ls /dev/tty*
    ```
    If your RPLIDAR is connected to a different port, you will need to update the `PORT_NAME` variable in the `main.py` script.
    หาก RPLIDAR ของคุณเชื่อมต่อกับพอร์ทอื่น คุณจะต้องแก้ไขค่าของตัวแปร `PORT_NAME` ในไฟล์ `main.py`

2.  **Grant port permissions (if needed):** / **ให้สิทธิ์การเข้าถึงพอร์ท (ถ้าจำเป็น):**
    You may need to grant your user permission to access the serial port. You can do this by adding your user to the `dialout` group:
    คุณอาจต้องให้สิทธิ์ผู้ใช้งานของคุณในการเข้าถึงพอร์ท โดยสามารถทำได้โดยการเพิ่มผู้ใช้ของคุณไปยังกลุ่ม `dialout`:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    **Important:** You will need to log out and log back in for this change to take effect. / **ข้อสำคัญ:** คุณต้องออกจากระบบและเข้าสู่ระบบใหม่เพื่อให้การเปลี่ยนแปลงนี้มีผล

3.  **Run the script:** / **รันสคริปต์:**
    Execute the main script to start scanning: / รันสคริปต์หลักเพื่อเริ่มการสแกน:
    ```bash
    python3 main.py
    ```

The script will: / สคริปต์จะทำงานดังนี้:
- Connect to the RPLIDAR. / เชื่อมต่อกับ RPLIDAR
- Print the device information (`get_info()`). / พิมพ์ข้อมูลของอุปกรณ์
- Print the device health status (`get_health()`). / พิมพ์สถานะของอุปกรณ์
- Start scanning and print the number of measurements received in each scan for a limited number of cycles. / เริ่มการสแกนและพิมพ์จำนวนค่าที่วัดได้ในแต่ละรอบ (ตามจำนวนรอบที่จำกัดไว้)

To stop the script, press `Ctrl+C`. The program will safely stop the LIDAR's motor and disconnect. / หากต้องการหยุดการทำงาน กด `Ctrl+C` โปรแกรมจะทำการหยุดมอเตอร์ของ LIDAR และตัดการเชื่อมต่ออย่างปลอดภัย