# RQT RPLIDAR Plot Plugin

This package provides an `rqt` plugin to visualize `sensor_msgs/LaserScan` data from an RPLIDAR in real-time. The plot is displayed as a polar chart, showing distances at different angles.

โปรเจคนี้คือ `rqt` plugin สำหรับแสดงผลข้อมูล `sensor_msgs/LaserScan` จาก RPLIDAR แบบ real-time โดยแสดงผลในรูปแบบกราฟเชิงขั้ว (polar plot) ซึ่งจะแสดงระยะทางที่วัดได้ในแต่ละมุม

---

## Architecture / สถาปัตยกรรม

This plugin is designed for stability and performance, especially when handling high-frequency sensor data.
ปลั๊กอินนี้ถูกออกแบบมาเพื่อความเสถียรและประสิทธิภาพ โดยเฉพาะเมื่อต้องจัดการกับข้อมูลจากเซ็นเซอร์ที่มีความถี่สูง

*   **GUI Framework:** The user interface is built with **PyQt5**, and the plotting is handled by **PyQtGraph**, a library optimized for high-performance, real-time scientific plotting.
    *   *GUI ถูกสร้างด้วย **PyQt5** และการพล็อตกราฟใช้ **PyQtGraph** ซึ่งเป็นไลบรารีที่ถูกออกแบบมาเพื่อการพล็อตทางวิทยาศาสตร์แบบ real-time ประสิทธิภาพสูงโดยเฉพาะ*
*   **ROS 2 Integration:** The plugin uses the ROS 2 node provided by the `rqt` context. This ensures proper integration and lifecycle management within the `rqt` framework.
    *   *ปลั๊กอินนี้ใช้ ROS 2 node ที่ `rqt` framework จัดการและส่งมาให้ ซึ่งเป็นวิธีมาตรฐานที่ทำให้การทำงานและจัดการ life cycle ถูกต้องสมบูรณ์*
*   **Real-time & Non-blocking UI:** The GUI is updated at a fixed rate (~30 Hz) using a `QTimer`. This reads the latest data received by the subscriber and redraws the plot, ensuring a smooth user experience without blocking the GUI thread.
    *   *GUI จะถูกอัปเดตด้วยความถี่คงที่ (~30 Hz) โดยใช้ `QTimer` ซึ่งจะดึงข้อมูลล่าสุดที่ subscriber ได้รับมาวาดใหม่ การแยกส่วนนี้ช่วยป้องกันไม่ให้ GUI ค้างและทำให้โปรแกรมทำงานได้อย่างราบรื่น*
*   **QoS Profile:** The subscriber uses the `qos_profile_sensor_data` profile, which is standard for sensor data, ensuring reliable communication.
    *   *Subscriber ใช้ `qos_profile_sensor_data` ซึ่งเป็นโปรไฟล์มาตรฐานสำหรับข้อมูลจากเซ็นเซอร์ เพื่อให้การสื่อสารมีความน่าเชื่อถือ*

## 1. Prerequisites / สิ่งที่ต้องมี

*   **Ubuntu 22.04** with **ROS 2 Humble Hawksbill** installed.
*   **colcon** build tools.
*   An RPLIDAR A1 (or compatible) running and publishing to the `/scan` topic. You can use the `rplidar_ros` package for this.
    *   *RPLIDAR A1 (หรือรุ่นที่เข้ากันได้) ที่กำลังทำงานและ publish ข้อมูลไปยัง topic `/scan` (สามารถใช้ `rplidar_ros` package เพื่อทำสิ่งนี้ได้)*

## 2. Installation / การติดตั้ง

### 2.1. Install Dependencies / ติดตั้ง Dependencies

First, ensure you have the necessary ROS 2 packages and Python libraries.
ก่อนอื่น ตรวจสอบให้แน่ใจว่าคุณได้ติดตั้ง package ของ ROS 2 และไลบรารีของ Python ที่จำเป็นแล้ว

```bash
sudo apt-get update
# Install rplidar driver, rqt, and python dependencies
sudo apt-get install ros-humble-rplidar-ros ros-humble-rqt python3-pyqt5 python3-pyqtgraph
```

### 2.2. Set Up a ROS 2 Workspace / ตั้งค่า Workspace ของ ROS 2

If you don't have a ROS 2 workspace, create one.
หากคุณยังไม่มี ROS 2 workspace ให้สร้างขึ้นมาใหม่

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2.3. Add This Package to Your Workspace / เพิ่ม Package นี้ไปยัง Workspace

Clone this repository into your workspace's `src` directory.
คัดลอกโปรเจคนี้ไปไว้ในไดเรกทอรี `src` ของ workspace

```bash
# Inside ~/ros2_ws/src
git clone <repository_url>
```
*(Note: Replace `<repository_url>` with the actual URL of this repository.)*
*(หมายเหตุ: แก้ `<repository_url>` เป็น URL ของโปรเจคนี้)*

### 2.4. Build the Workspace / Build Workspace

Navigate to the root of your workspace and build the package.
ไปยังไดเรกทอรีหลักของ workspace แล้ว build package

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## 3. Usage / การใช้งาน

### 3.1. Source Your Workspace / Source Workspace ของคุณ

Each time you open a new terminal, source your ROS 2 environment and your workspace.
ทุกครั้งที่เปิด terminal ใหม่ คุณต้อง source สภาพแวดล้อมของ ROS 2 และ workspace ของคุณ

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 3.2. Run the RPLIDAR Driver / รันไดรเวอร์ RPLIDAR

In one terminal, start the RPLIDAR node.
ใน terminal แรก, เริ่มการทำงานของ RPLIDAR node

```bash
# Don't forget to grant permissions to /dev/ttyUSB0 first!
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### 3.3. Run the RQT Plugin / รัน RQT Plugin

In a second terminal, you can run the plugin in two ways:
ใน terminal ที่สอง, คุณสามารถรัน plugin ได้ 2 วิธี:

**Run the RQT Plugin / รัน RQT Plugin**

1.  Run `rqt`:
    ```bash
    rqt
    ```
2.  From the menu, navigate to `Plugins` > `Visualization` > `RPLIDAR Plot`.

You should now see a window displaying the real-time scan data from your RPLIDAR.
คุณจะเห็นหน้าต่างที่แสดงผลข้อมูลการสแกนจาก RPLIDAR ของคุณแบบ real-time