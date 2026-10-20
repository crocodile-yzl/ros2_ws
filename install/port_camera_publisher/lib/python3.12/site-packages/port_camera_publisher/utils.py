"""节点共用的小工具。"""

from rcl_interfaces.msg import ParameterDescriptor


def dynamic_descriptor() -> ParameterDescriptor:
    """允许命令行覆盖值改变类型。

    命令行 `-p source:=0` 或 `-p camera_fps:=30` 会被 ROS 按 YAML 解析成
    INTEGER，而参数若按 STRING/FLOAT 固定声明，rclpy 会直接抛
    InvalidParameterTypeException。用动态类型声明，读取时再自行转换。
    """
    return ParameterDescriptor(dynamic_typing=True)


def as_bool(value) -> bool:
    """把 true/false/1/0 这类字符串也当布尔处理。"""
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)
