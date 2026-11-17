# 本项目单个舵机的一次查询结果；状态常量与字段一同生成到消息类型中。
uint8 NOT_QUERIED=0
uint8 OK=1
uint8 FAILED=2

# 当前项目的舵机地址000～005，以数字0～5传输。
uint8 servo_id
uint8 status

# 只有status==OK时有效，单位为协议P，不是弧度或毫米。
# 失败/未查询时的0是占位初值，不能作为实物位置使用。
uint16 position_p

# native查询函数调用的主机耗时，单位毫秒，不是舵机声明的运动T。
float64 elapsed_ms
# 本轴函数结束相对批次开始的主机毫秒偏移，不是设备采样时间戳。
float64 finished_offset_ms
# 失败原因；未查询/成功时通常为空。
string error
