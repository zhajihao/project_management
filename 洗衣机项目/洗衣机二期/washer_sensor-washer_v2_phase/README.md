Washer Project - 洗衣机2期项目

1.创建工程目录

2.添加咖啡机的ble程序
  添加洗衣机2期的MCU工程

3.根据与APP联调情况修改部分代码
   添加与APP的通信协议

4.修复eeprom写操作时的BUG

5.蓝牙和mcu工程修改：
--由于订单编号中存在随机数，所以蓝牙判断包尾去掉0x0a或0x0d判断。
--修改蓝牙判断方式为20字节一包数据，MCU发送数据去掉回车换行，并增加字节数为20。
--mcu去掉扫描后的1分钟超时判断。

6.修改蓝牙名称为“TOSEI_sensor”

7.MCU添加洗衣机门未关闭的状态，修改部分判断逻辑