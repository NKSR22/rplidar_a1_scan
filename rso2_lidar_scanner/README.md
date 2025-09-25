# RSO2 RPLIDAR Scanner

This ROS 2 package provides a convenient way to launch the `rplidar_ros` driver for RPLIDAR A1 and includes a simple subscriber node to verify that data is being published correctly.

โปรเจคนี้คือ ROS 2 package สำหรับช่วยให้การเปิดใช้งาน `rplidar_ros` driver สำหรับ RPLIDAR A1 ทำได้สะดวกขึ้น พร้อมทั้งมี subscriber node สำหรับตรวจสอบว่าข้อมูลถูกส่งออกมาอย่างถูกต้อง

---

## 1. Prerequisites / สิ่งที่ต้องมี

*   **Ubuntu 22.04** with **ROS 2 Humble Hawksbill** installed.
    *   *Ubuntu 22.04 ที่ติดตั้ง ROS 2 Humble Hawksbill เรียบร้อยแล้ว*
*   **colcon** build tools.
    *   *เครื่องมือสำหรับ build ที่ชื่อว่า colcon*
*   An RPLIDAR A1 connected to a USB port.
    *   *เซ็นเซอร์ RPLIDAR A1 ที่เชื่อมต่อกับพอร์ต USB*

## 2. Installation / การติดตั้ง

### 2.1. Install RPLIDAR ROS 2 Driver / ติดตั้งไดรเวอร์ RPLIDAR สำหรับ ROS 2

The official `rplidar_ros` package can be installed directly using `apt`. This is the recommended method.
เราสามารถติดตั้ง `rplidar_ros` package ผ่าน `apt` ได้โดยตรง ซึ่งเป็นวิธีที่แนะนำ

```bash
sudo apt-get update
sudo apt-get install ros-humble-rplidar-ros
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
*(Note: Replace `<repository_url>` with the actual URL of this repository. If you downloaded the files, just place the `rso2_lidar_scanner` folder here.)*
*(หมายเหตุ: แก้ `<repository_url>` เป็น URL ของโปรเจคนี้ หรือหากดาวน์โหลดมาเป็นไฟล์ ก็ให้นำโฟลเดอร์ `rso2_lidar_scanner` มาวางไว้ที่นี่)*

### 2.4. Build the Workspace / Build Workspace

Navigate to the root of your workspace and build the package using `colcon`.
ไปยังไดเรกทอรีหลักของ workspace แล้ว build package ด้วย `colcon`

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## 3. Configuration / การตั้งค่า

### 3.1. Grant Port Permissions / ให้สิทธิ์การเข้าถึงพอร์ท

To allow ROS 2 to access the RPLIDAR, you need to grant permissions to the USB port (usually `/dev/ttyUSB0`).
เพื่อให้ ROS 2 สามารถเข้าถึง RPLIDAR ได้ คุณต้องให้สิทธิ์การเข้าถึงพอร์ต USB (โดยทั่วไปคือ `/dev/ttyUSB0`)

A permanent solution is to add your user to the `dialout` group:
วิธีที่ถาวรคือการเพิ่มผู้ใช้ของคุณไปยังกลุ่ม `dialout`

```bash
sudo usermod -a -G dialout $USER
```
**Important:** You must **log out and log back in** for this change to take effect.
**ข้อสำคัญ:** คุณต้อง **ออกจากระบบและเข้าสู่ระบบใหม่** เพื่อให้การเปลี่ยนแปลงนี้มีผล

Alternatively, the `rplidar_ros` package provides a script to create a udev rule, which is an even better method.
อีกทางเลือกหนึ่ง ซึ่งเป็นวิธีที่ดีกว่า คือการใช้สคริปต์ที่มากับ `rplidar_ros` เพื่อสร้าง udev rule

```bash
ros2 run rplidar_ros create_udev_rules.sh
```

## 4. Usage / การใช้งาน

### 4.1. Source Your Workspace / Source Workspace ของคุณ

Each time you open a new terminal, you need to source your ROS 2 environment and your workspace's setup file.
ทุกครั้งที่เปิด terminal ใหม่ คุณต้อง source สภาพแวดล้อมของ ROS 2 และไฟล์ setup ของ workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```
*(Tip: Add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` file to do this automatically.)*
*(คำแนะนำ: เพิ่ม `source ~/ros2_ws/install/setup.bash` ไปยังไฟล์ `~/.bashrc` เพื่อให้คำสั่งนี้รันโดยอัตโนมัติ)*

### 4.2. Launch the Scanner / รันโปรแกรมสแกนเนอร์

Use `ros2 launch` to start the RPLIDAR driver and the subscriber node.
ใช้ `ros2 launch` เพื่อเริ่มการทำงานของไดรเวอร์ RPLIDAR และ subscriber node

```bash
ros2 launch rso2_lidar_scanner start_scanner_launch.py
```

You should see output from both the `rplidar_node` and our `scanner_subscriber`. The subscriber will print messages confirming that it is receiving `LaserScan` data.
คุณจะเห็นข้อความจาก `rplidar_node` และ `scanner_subscriber` ของเรา โดย subscriber จะพิมพ์ข้อความเพื่อยืนยันว่าได้รับข้อมูล `LaserScan` แล้ว