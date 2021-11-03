# SLAS-VLP-in-ROS

单灯定位文件夹(single_led_simple [public version])

PS:注意事项 当电脑运行太久卡顿时,无法通过Ctrl+C结束时,Ctrl+Alt+T重开一个终端 查看指定进程 ps -ef|grep 关键词 如: ps -ef|grep single_led_locator 查看到当前运行的进程号 xxx 再通过下述命令杀掉对应的进程 kill -9 xxx

相机功能包（mvcam）有以下节点: mvcam节点----实时获取拍摄到的图像

单灯定位功能包（single_led)有以下节点: point_subscriber ----订阅显示接收到的定位坐标结果(坐标单位是cm)

//多线程

single_led_locator_multi_spin_imu_simple----订阅接收到的角度和图像信息,进行定位计算(imu+image)
single_led_locator_multi_spin_imu_mag_simple----订阅接收到的角度和图像信息,进行定位计算(imu+mag+image)
single_led_locator_multi_spin_odom_simple----订阅接收到的角度和图像信息,进行定位计算(odom+image)
single_led_locator_multi_spin_imu_mag_filter_simple----订阅接收到的角度（松同步）和图像信息,进行定位计算(imu+mag+image)
single_led_locator_multi_spin_mag_simple----订阅接收到的角度和图像信息,进行定位计算(mag)

//角度对比节点 single_led_locator_imu_mag_odom_simple

//img_subscriber ----订阅显示接收到的图像(测试节点,实际操作时无需运行) 

//imu_subscriber ----订阅显示接收到的imu数据 (测试节点,实际操作时无需运行)
