这是一个使用语音交流进行拍照和颜色识别的程序包。

功能设想：

1.在之前的语音交互功能基础上，添加拍照和颜色识别功能。

2.拍照

机器人调用人脸检测结点face_detection检测人脸的位置，根据获得的位置判断人脸中心是否在摄像头中心，若不在则机器人发声引导被摄像者调整位置，直到
位置合适，机器人调用take_photo结点进行拍照，并将结果显示。

3.颜色识别

机器人调用camshift结点对某种颜色（蓝色）进行识别和追踪，每次命令机器人会尝试识别5次。

使用结点：

lm_control  : 加载语音模块包

audio_control  : 进行语音识别

soundplaynode  : 实现语音输出


usb_cam  ：调用usb摄像头

take_photo ： 通过接收topic的指定消息实现拍照功能

face_detection  ： 进行人脸检测

camshift  ： 对颜色进行识别和追踪

注：上述结点依赖于ros-kinetic-usb-cam包 

               opencv_apps包 ： https://github.com/ros-perception/opencv_apps

talkbot  : 主结点，调用上述结点实现人机语音交互，并完成相关功能
