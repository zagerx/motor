# motor
- motor模块以FOC为核心
- modulestest用来测试motor模块
- 开发板配置
- 主控配置
- FOC控制接口
- pwm模块(定时器发波)
    - 定时器启动
    - 定时器停止
    - 定时器发波
- currsmap电流采样模块
    - adc配置
- 编码器模块
    - 定时器启动
    - 定时器停止
- 模式控制
- 故障检测模块

- `west build -b zgm_002 motor/ -- -DBOARD_ROOT=$(pwd)/motor -DOVERLAY_CONFIG="superlift_app.conf"`