#  mpu6050



quaterninon data for geometry_msgs/msg/Quaternion

H/W : 아두이노 나노와  GY-521

S/W : ROS2 Humble + mpu6050dmp

### 1. MPU6050 아두이노 라이브러리 설치 및 수정

![image](https://github.com/firstbot1/imu_serial/assets/88788285/e5a935de-372a-41e5-b5c2-62e0b85b0bb6)


1.1 MPU6050예제 중에서 MPU6050_DMP를 아래와 같이 수정

a. #define OUTPUT_READABLE_QUATERNION 주석 해제 quaterninon 사용.

    geometry_msgs/msg/Quaternion 메시지를 publish하기 위해....

b. 시리얼 전송 속도는 57600bps

c. 시리얼 전송 메시지는 ","구분하고 100ms 마다 전송하기 위해 아래와 같이 수정.

```
unsigned long previousMillis = 0;
const long interval = 100; // 100ms = 10Hz
void loop() {

    unsigned long currentMillis = millis();

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            
            if (currentMillis - previousMillis >= interval) {
              // 이전에 이벤트를 실행한 이후 interval 시간이 지났을 때
              previousMillis = currentMillis;
              Serial.print("quat");
              Serial.print(",");
              Serial.print(q.w);
              Serial.print(",");
              Serial.print(q.x);
              Serial.print(",");
              Serial.print(q.y);
              Serial.print(",");
              Serial.println(q.z);
              
            }
            
        #endif
```

위와 같이 수정한 코드를 GY-521를 아두이노에 연결 한 후 업로드 한다.

quaternion은 복소수를 확장한 개념으로 3D 공간에서의 회전을 나타내는 데 특히 유용한 수학적 표현입니다. 기본적으로, quaternion은 네 개의 스칼라 값 w, x, y, z로 구성되며, 이는 일반적으로 q=w+xi+yj+zk 형태로 표현됩니다. 여기서 i, j, k는 quaternion의 기본 단위입니다.

\* w - 스칼라 부분이며, 회전의 "양" 또는 "크기"를 나타냅니다.

\* x, y, z - 벡터 부분이며, 회전의 "축"을 나타냅니다.

quaternion은 아래와 같은 특징을 가진다.

1. **짐벌 락(Gimbal Lock) 문제 없음**: 오일러 각도와 같은 다른 회전 표현 방식은 짐벌 락이라는 문제점을 가지고 있습니다. quaternion은 이 문제를 방지합니다.
2. **보간이 간단함**: 두 회전 사이의 보간을 수행할 때 quaternion은 슬러프(SLERP)라는 기법을 사용하여 간단하고 부드러운 결과를 얻을 수 있습니다.
3. **합성이 효율적**: 두 개의 quaternion 회전을 합성할 때, 단순한 행렬 연산보다 연산 비용이 적습니다.

그러나 quaternion은 직관적이지 않은 표현을 가지므로, 일반적으로 애플리케이션에서 직접 사용자에게 표시하기보다는 내부 연산에 사용됩니다.

아두이노의 MPU6050 DMP 코드에서 **mpu.dmpGetQuaternion(&q, fifoBuffer);**는 MPU6050의 DMP(Digital Motion Processor)로부터 quaternion 값을 가져옵니다. 이 quaternion 값은 센서의 현재 3D 공간에서의 회전 상태를 나타냅니다. 여기서 **fifoBuffer**는 DMP에서 가져온 원시 데이터 버퍼이며, **&q**는 가져온 quaternion 값을 저장할 구조체의 참조입니다.



### 2. 패키지생성 및 노드 작성**

2.1 workspace 생성 

```
mkdir -p ~/ros2_ws/src cd ~/ros2_ws/
```

2.2 패키지 생성

```
cd ~/ros2_ws/src ros2 pkg create --build-type ament_python imu_serial
```

2.3 노드 코드 추가 < mpu6050_dmp.py >

```
cd ~/ros2_ws/src/imu_serial/imu_serial touch mpu6050_dmp6.py
```

<  mpu6050_dmp.py > 노드를 아래와 같이 작성한다.

```
# ROS 2와 필요한 패키지들을 임포트합니다.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import serial  # 시리얼 통신을 위한 라이브러리입니다.

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_quaternion_node')
        
        # 쿼터니언 데이터를 위한 퍼블리셔를 생성합니다.
        self.pub = self.create_publisher(QuaternionStamped, 'imu_quaternion', 10)
        
        # 시리얼 통신을 설정합니다. 
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.ser.flush()

        # tf2를 위한 브로드캐스터를 생성합니다.
        self.br = TransformBroadcaster(self)
        
        # 주기적으로 callback 함수를 호출하는 타이머를 생성합니다.
        self.timer = self.create_timer(0.01, self.callback)  # 100Hz

    def callback(self):
        # 시리얼 버퍼에 데이터가 있을 경우 읽어옵니다.
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            data = line.split(",")
            
            # 데이터의 정합성을 체크합니다.
            if len(data) != 5 or not data[0] == "quat":
                self.ser.flushInput()  # 버퍼를 비워줍니다.
                return
            
            try:
                # 쿼터니언 메시지를 생성합니다.
                quat_stamped = QuaternionStamped()
                quat_stamped.header.stamp = self.get_clock().now().to_msg()
                quat_stamped.header.frame_id = "imu_frame"
                quat_stamped.quaternion.w = float(data[1])
                quat_stamped.quaternion.x = float(data[2])
                quat_stamped.quaternion.y = float(data[3])
                quat_stamped.quaternion.z = float(data[4])
                    
                # 쿼터니언 메시지를 발행합니다.
                self.pub.publish(quat_stamped)

                # tf2 변환 정보를 설정하고 발행합니다.
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "base_link"
                t.child_frame_id = "imu_frame"
                t.transform.rotation = quat_stamped.quaternion
                self.br.sendTransform(t)

            except Exception as e:
                # 오류 발생 시 로깅합니다.
                self.get_logger().info("Error parsing data: " + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 시리얼 통신을 닫고 노드와 rclpy를 종료합니다.
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

이 코드는 ROS 2를 사용하여 IMU(Inertial Measurement Unit)의 데이터를 시리얼 통신으로 받아와서 ROS 메시지로 전송하고, tf2(transform)를 사용하여 이 정보를 시각화할 수 있는 형태로 변환하는 역할을 합니다.



SerialNode 클래스는 ROS 2 노드를 정의하며, 초기화 과정에서는 노드 이름을 'serial_quaternion_node'로 설정합니다.

시리얼 포트 /dev/ttyUSB0에서 57600의 바우드레이트로 통신을 시작합니다.

'imu_quaternion'이라는 이름의 QuaternionStamped 메시지를 위한 퍼블리셔를 생성합니다.

tf2 변환을 위한 브로드캐스터도 초기화합니다.

0.01초 간격(100Hz)으로 callback 함수를 호출하기 위한 타이머를 설정합니다.



callback 함수는 타이머에 의해 주기적으로 호출되며, 이 함수에서는 시리얼 버퍼에서 데이터를 수신합니다.

수신된 데이터는 콤마로 구분된 문자열이며, "quat"라는 시작 토큰과 함께 4개의 쿼터니언 값을 포함해야 합니다.

데이터가 올바르게 형식화되어 있지 않으면 버퍼는 비워지며 처리가 종료됩니다.

올바르게 형식화된 데이터의 경우, 이를 파싱하여 QuaternionStamped 메시지를 생성하고 발행합니다.

동시에, 이 쿼터니언 정보를 tf2 변환 정보로 변환하여 TransformStamped 메시지를 생성하고 브로드캐스터를 사용하여 전송합니다.

메인 함수:



ROS 2 노드를 초기화하고, SerialNode 인스턴스를 생성합니다.

rclpy.spin(node)를 통해 노드가 실행되며, 이는 노드가 종료될 때까지 계속됩니다.

마지막으로, 노드가 종료되면 시리얼 통신 연결을 닫고 노드와 rclpy를 종료합니다.

결론적으로, 이 코드는 시리얼 포트에서 IMU 데이터를 수신하고, 해당 데이터를 ROS 2의 메시지와 tf2 변환으로 변환하여 ROS 2 시스템에서 사용할 수 있게 합니다.



### 2.4 package.xml 수정

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>imu_serial</name>
  <version>0.0.1</version> <!-- 패키지 버전. 일반적으로 초기 버전을 0.0.1로 설정합니다. -->
  <description>A ROS2 package to read and publish IMU data from a serial port.</description> <!-- 패키지 설명을 간략하게 작성했습니다. -->
  <maintainer email="firstbot@naver.com">firstbot</maintainer>
  <license>Apache License 2.0<</license> 

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```



### 2.5 setup.py를 수정

```
from setuptools import find_packages, setup

package_name = 'imu_serial'

setup(
    name=package_name,
    version='0.0.1',  # 패키지 버전. 일반적으로 초기 버전을 0.0.1로 설정합니다.
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='firstbot',
    maintainer_email='firstbot@naver.com',
    description='A ROS2 package to read and publish IMU data from a serial port.',  # 패키지 설명을 간략하게 작성했습니다.
    license='Apache-2.0',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_dmp6 = imu_serial.mpu6050_dmp6:main',  # 노드를 실행 가능한 스크립트로 추가합니다.
        ],
    },
)
```



### 2.6 빌드 및 실행

```
cd ~/ros2_ws/
colcon build
```

```
source install/setup.bash
ros2 run imu_serial mpu6050_dmp6
```

topic이 정상적으로 발행 되는지 학인.

<img src="/home/firstbot/Pictures/1a_2.png" alt="이미지 대체 텍스트" style="float: left;">

ros2 topic echo /imu_quaternion을 시행 정상적으로 quaterinon 토픽이 발행 되는지 확인.

<img src="/home/firstbot/Pictures/1a_3.png" alt="이미지 대체 텍스트" style="float: left;">

tf2 tree 확인 

<img src="/home/firstbot/Pictures/1a_4.png" alt="이미지 대체 텍스트" style="float: left;">

해당 그래프에 대한 설명은 다음과 같습니다:

1. **노드**:

- **"base_link"**: 시작 프레임 또는 기준 프레임을 나타냅니다.
- **"imu_frame"**: 대상 프레임을 나타냅니다.

1. **엣지 (화살표)**:

- **"base_link"**에서 **"imu_frame"**으로의 화살표는 두 프레임 사이의 변환을 나타냅니다. 해당 변환의 정보는 라벨에서 제공됩니다:
- **Broadcaster**: 변환을 발행하는 노드나 엔터티. 여기서는 **default_authority**입니다.
- **Average rate**: 변환의 평균 발행 속도, 여기서는 약 9.859Hz입니다.
- **Buffer length**: 변환의 버퍼 길이, 여기서는 5.071입니다.
- **Most recent transform**: 가장 최근 변환의 타임스탬프.
- **Oldest transform**: 버퍼에 저장된 가장 오래된 변환의 타임스탬프.

1. **서브그래프 (subgraph)**:

- **"view_frames Result"**: 해당 그래프의 제목 또는 범례를 나타냅니다.
- **"Recorded at time: 1696687911.5345802"**: **tf2** 트리가 기록된 시간을 나타냅니다.

이 그래프는 **tf2** 시스템의 상태와 **base_link**와 **imu_frame** 사이의 변환에 대한 정보를 제공합니다. Graphviz 도구를 사용하면 이 텍스트를 시각적인 그래프 형태로 표현할 수 있습니다.



### 2.7 Visualization

```
ros2 run rviz2 rviz2
```

<img src="/home/firstbot/Pictures/1a_5.png" alt="이미지 대체 텍스트" style="float: left;">

생각보다는 꽤 정확한것 같다. 내가 mpu6050 imu를  테스트해보는 것은 odom wheel 대신 laser_scan과 fusion하여 사용할 수 있는지 확인해 보기 위해서 이다. 또한 로봇팔에도 적용이 가능한지 확인해 보기 위해서 그 정확성이 궁금해서이고 arduino 라이브러리인 mpu6050에 포함된 dmp6 알고리즘은 관련 알고리즘들 중에서  가장 정확한것 같다.
