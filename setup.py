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
    maintainer_email='hyjeong7212@naver.com',
    description='A ROS2 package to read and publish IMU data from a serial port.',  # 패키지 설명을 간략하게 작성했습니다.
    license='BSD',  # BSD 라이선스로 설정했습니다. 필요한 라이선스로 변경해주세요.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_dmp6 = imu_serial.mpu6050_dmp6:main',  # 노드를 실행 가능한 스크립트로 추가합니다.
        ],
    },
)
