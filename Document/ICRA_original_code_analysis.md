#### ICRA代码架构

![img](https://imgconvert.csdnimg.cn/aHR0cHM6Ly9ybS1zdGF0aWMuZGppY2RuLmNvbS9kb2N1bWVudHMvMjA3NTgvZjQyZDY1ZDg1ZDk3YzE1NDc1NTMxMDY1Mzk3ODM2MDYucG5n?x-oss-process=image/format,png)

​	

#### 功能实现：

| 包的名称             | 功能               | 内部依赖                                                     |
| -------------------- | ------------------ | ------------------------------------------------------------ |
| roborts              | Meta-package       | /                                                            |
| roborts_base         | 嵌入式通信接口     | roborts_msgs                                                 |
| roborts_camera       | 相机驱动           | roborts_common                                               |
| roborts_common       | 通用依赖包         | /                                                            |
| roborts_decision     | 机器人决策         | roborts_common<br>roborts_msgs<br>roborts_costmap            |
| roborts_detection    | 视觉识别           | roborts_msgs<br>roborts_common<br>roborts_camera             |
| roborts_localization | 机器人定位算法     | /                                                            |
| roborts_costmap      | 代价地图相关支持包 | roborts_common                                               |
| roborts_msgs         | 自定义消息类型     | /                                                            |
| roborts_planning     | 运动规划算法       | roborts_common<br>roborts_msgs<br>roborts_costmap            |
| roborts_bringup      | 启动包             | roborts_base<br>roborts_common<br>roborts_localization<br>roborts_costmap<br>roborts_msgs<br>roborts_planning |
| roborts_tracking     | 视觉追踪算法       | roborts_msgs                                                 |

#### roborts_camera

 主要调用的部分为**camera_node.cpp**（用于初始化相机，调用相机，等待图片返回）运用

```c++
int main(int argc, char **argv){
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "roborts_camera_node", ros::init_options::NoSigintHandler);//初始化机器人相机节点
  roborts_camera::CameraNode camera_test;// 设置一个名为camera_test的相机节点
  ros::AsyncSpinner async_spinner(1);// 同时1个线程进行接收数据
  async_spinner.start();// 开始接收
  ros::waitForShutdown();
  return 0;
```

#### roborts_dectection

用于装甲板检测

1. **armor_detection_client.cpp**等待接收camera发送的图片消息，接收到nh_节点，然后进行装甲板检测

2. **armor_detection_node.cpp**在该文件中实现了对敌方装甲板的检测和调整云台姿态（roll, pitch, yaw），其中实现的流程为：

   ```c++
   void ArmorDetectionNode::ExecuteLoop() {
     undetected_count_ = undetected_armor_delay_;// 未找到敌人时的延迟
   
     while(running_) {// 循环执行
       usleep(1);
       if (node_state_ == NodeState::RUNNING) {// 运行状态下，执行下列程序
         cv::Point3f target_3d;
         ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, target_3d);// 如果运行过程中报错
         {
           std::lock_guard<std::mutex> guard(mutex_);// 进行警戒
           x_ = target_3d.x;// 目标的位置
           y_ = target_3d.y;
           z_ = target_3d.z;
           error_info_ = error_info;// 错误信息
         }
   
         if(detected_enemy_) {// 如果找到了敌人
           float pitch, yaw;// 给定pitch和yaw角度
           gimbal_control_.Transform(target_3d, pitch, yaw);// 控制云台到达指定位置
   
           gimbal_angle_.yaw_mode = true;
           gimbal_angle_.pitch_mode = false;
           gimbal_angle_.yaw_angle = yaw * 0.7; // 调整优化yaw的角度（为什么是0.7，后续研究）
           gimbal_angle_.pitch_angle = pitch;
   
           std::lock_guard<std::mutex> guard(mutex_);
           undetected_count_ = undetected_armor_delay_;// 设定未发现装甲延迟
           PublishMsgs();// 发布云台角度
         } else if(undetected_count_ != 0) {// 如果延迟不为0,则调整云台状态
   
           gimbal_angle_.yaw_mode = true;
           gimbal_angle_.pitch_mode = false;
           gimbal_angle_.yaw_angle = 0;
           gimbal_angle_.pitch_angle = 0;
   
           undetected_count_--;
           PublishMsgs();// 发布云台角度
         }
       } else if (node_state_ == NodeState::PAUSE) {// 暂停执行，调整各个节点的状态
         std::unique_lock<std::mutex> lock(mutex_);
         condition_var_.wait(lock);
       }
     }
   }
   ```

3. **（重点）gimbal_control.cpp**云台控制。包含云台的位置，pitch、yaw轴以及v和k。这个文件中包含了控制pitch角度、计算发射后子弹高度和姿态变换计算三种方法。

   ```c++
   //air friction is considered
   float GimbalContrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
     float t, y;
     t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));// 计算子弹飞行时间
     y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);// 子弹打中时的理想高度
     return y;
   }
   
   //x:distance , y: height
   float GimbalContrl::GetPitch(float x, float y, float v) {
     float y_temp, y_actual, dy;
     float a;
     y_temp = y;
     // by iteration
     for (int i = 0; i < 20; i++) {
       a = (float) atan2(y_temp, x);// pitch角度
       y_actual = BulletModel(x, v, a);// 计算a角度下子弹打中的实际高度
       dy = y - y_actual;// 需要调整的角度大小
       y_temp = y_temp + dy;// 最终预期角度
       if (fabsf(dy) < 0.001) {
         break;
       }
     }
     return a;
   }
   
   void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw) {
     pitch =
         -GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, 15) + (float)(offset_pitch_ * 3.1415926535 / 180);
     //yaw positive direction :anticlockwise
     yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);
   }// todo：确定各个参数和验证运算式的可靠性
   ```

   

4. **（重点）**在armor_detection_algorithms.h中调用了**constrain_set.cpp**，因此实现装甲板检测的算法在该cpp文件中。通过检测灯条->过滤掉部分灯条->列出可能存在的装甲板->过滤掉部分平均值标准差过大的装甲板->选取最终的装甲板->计算该装甲板的信息->解算装甲板坐标和进行信号滤波。

#### roborts_localization

用于小机器人定位，用ros中的TF功能包和localization_math.cpp中的运算方式计算机器人位姿

1. **localization_math.cpp**中使用多步运算，进行位姿运算，具体的计算方式需要进一步了解
2. 调用传感器进行定位，**localization_node.cpp**中包含多个方法（如：初始化、获取静态地图、多个坐标间TF变换等），此处不一一展开，下面为其中的main方法。

```c++
int main(int argc, char **argv) {
  roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "localization_node");// 初始化定位节点
  roborts_localization::LocalizationNode localization_node("localization_node");// 进行定位
  ros::AsyncSpinner async_spinner(THREAD_NUM);// 根据情况定义线程数量
  async_spinner.start();// 开始运行（循环）
  ros::waitForShutdown();// 一直运行，直到接收停止信息
  return 0;
}
```

