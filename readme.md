# 植物管家

本次项目设计一款智能花盆产品——植物管家。该花盆可通过环境温湿度传感器、土壤湿度传感器、光照传感器等对植物的生长环境的各项数据进行实时监控，利用 ARMAX 算法及实时网络数据结合植物信息库进行水量、肥料、光照的预测，从而实现光照、补充、光照的补充等相应措施，提高植物生存能力和质量，同时还可以通过WIFI无线传输将采集数据传回控制终端，实现植物生长的远程检测与控制。

# Introduction

## Video link

[演示视频-优酷视频链接](http://v.youku.com/v_show/id_XMzYzMTcxMTk1Ng==.html?spm=a2h3j.8428770.3416059.1)

## Function

项目在ARC（V2.2版本，内核ARC EM7D）开发板上综合利用传感技术、ARMAX算法(时间序列预测技术)、规则库设计、以及借助机智云平台进行数据传输和实时检测与控制，最终在移动通信设备上实现人机交互目的。通过土壤湿度、环境温湿度、光照等传感器采集实时数据，通过ARMAX模型及规则库确定盆栽的最佳浇灌时间和适宜的用水量，实现精准浇灌。同时将当前的传感器数据上传到机智云平台，通过WIFI无线传输到手机客户端使客户实时了解盆栽的生活状态。

## System Architecture

![System Architecture Photo](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/System-Architecture.png)

## UI

![UI Photo](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/UI.png)

# Hardware AND Software

## Hardware

|硬件模块|数据传输类型|数据引脚数|金额|
|:----:|:----:|:----:|:----:|
|电容式土壤传感器|模拟信号|1|7.85|
|DHT22|数字单总线|1|13.00|
|BH1750FVI光照模块|IIC总线|3|6.24|
|ADC|数字单总线|2|7.60|
|WiFi模块|串口通信|2|9.30|
|光电式水位传感器|数字单总线|1|1.44|
|水泵|——|——|9.80|
|继电器|——|4通道|8.05|
|电源模块|——|——|4.72|
|LED|——|——|1.00|
|LCD|IIC总线|2|21.9|
|亚克力板|——|——|56.00|

## Software

![Soft Call Photo](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/call.png)

本次代码中对emsk_init.c文件中的配置作了如下修改：

![Code Photo](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/code.png)

## Hardware Connection

![Hardware Photo1](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/Hardwarede1.png)

![Hardware Photo2](https://raw.githubusercontent.com/xdlp/SmartPlant/master/blobs/Hardwarede2.png)