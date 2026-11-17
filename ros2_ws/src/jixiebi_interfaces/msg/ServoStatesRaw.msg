# ROS节点在整批查询完成后的时间；不是设备采样时间，六轴也不是同时读取。
builtin_interfaces/Time stamp
# SERIAL_TTY标记真实TTY设备，不是设备路径；PTY_SIMULATION只代表本地伪终端测试。
string source
# 本节点进程内的批次序号，重启后重新计数，不是舵机协议request ID。
uint64 sequence

# 当前顺序查询固定为false；不能由批次共享stamp推断同步采样。
bool simultaneous
# 全部六个样本成功才为true；否则允许携带此前成功的部分结果。
bool complete
# 成功样本数0～6。
uint8 valid_count
# native整批查询耗时，单位毫秒。
float64 elapsed_ms

# 数组按舵机地址0～5排列；每个position_p是否可用仍须检查对应status。
ServoStateRaw[6] servos
