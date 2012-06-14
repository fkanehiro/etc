import rtm

rtc1 = rtm.findRTC("simple_vehicle")
rtc2 = rtm.findRTC("CameraImageViewer0")

rtm.connectPorts(rtc1.port("VISION_SENSOR1"),  rtc2.port("imageIn"))

rtc2.start()
