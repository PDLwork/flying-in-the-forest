import airsim

if __name__ == "__main__":
    client = airsim.MultirotorClient("192.168.1.109")  # 与airsim创建链接
    client.confirmConnection()  # 查询是否建立连接
    client.enableApiControl(True)   # 打开API控制权
    client.armDisarm(True)  # 解锁

    client.takeoffAsync().join()   # 起飞
    client.moveToZAsync(-3, 2).join()   # 上升到15m高度

    drivetrain = airsim.DrivetrainType.ForwardOnly
    yaw_mode = airsim.YawMode(False, 0)

    # 由于转换过坐标系，因此在这里坐标系就变成了 Y -x
    client.moveToPositionAsync(0, 5, -3, 1, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（10,0）点坐标
    client.moveToPositionAsync(0, 0, -3, 1, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 回到（0,0）点坐标

    # client.goHomeAsync().join()
    client.moveToZAsync(-1, 2).join()

    client.landAsync().join()     # 降落

    client.armDisarm(False)     # 上锁
    client.enableApiControl(False)   # 关闭API控制权

    print('测试完成')