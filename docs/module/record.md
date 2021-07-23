## VisionRecord Function(视觉录像功能)



>视觉<->电控控制录像功能已基本实现
### (1) 录制文件的默认保存位置为<u>/根目录/configs/record/record_packeg/....avi</u>
### (2) 默认模式为电控信号控制 串口发送Mode为<u>5</u>时进行录制，切换模式时保存。再次切换回5时再次在当前目录创建新的视频文件
### (3) 录制前需修改record_model.xml文件下的开关<br>
>RECORD_SWITCH
### (3) 视觉控制需要更改record_model.xml文件下的视觉控制开关选项 <br> 
>"VISION_ELECTRONIC_LOCK"
### (4) 录制方法(视觉模式下) <u>'s'</u>按键为开始录制，<u>'e'</u>为录制结束。视频文件路径和默认路径相同。<br><br><br>



#### 待优化处
>视频录制的帧率提升<br>
>Wolfvision.cpp文件的更新优化