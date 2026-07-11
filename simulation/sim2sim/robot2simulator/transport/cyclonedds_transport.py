import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import sys

from sim2sim.robot2simulator.joint_map import MotorHeader
from sim2sim.robot2simulator.motion_messages import MotionCtrl, MotionFault, MotionFaultItem, MotionObserved, MotionRecord
from sim2sim.robot2simulator.transport.base import MotionTransport


try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.core import Listener
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication, BuiltinTopicDcpsSubscription
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl import types
    from cyclonedds.qos import Policy, Qos
except Exception as e:  # pragma: no cover
    DomainParticipant = None
    DataWriter = None
    DataReader = None
    Topic = None
    Listener = None
    BuiltinDataReader = None
    BuiltinTopicDcpsPublication = None
    BuiltinTopicDcpsSubscription = None
    IdlStruct = object
    types = None
    Policy = None
    Qos = None
    _cyclonedds_import_error = e


def _require_cyclonedds() -> None:
    if DomainParticipant is None:  # pragma: no cover
        raise RuntimeError(
            "未检测到 Python 包 `cyclonedds`，无法直接在 Python 里做 DDS pub/sub。\n"
            "建议：在你的仿真 Python 环境中安装 `cyclonedds`，然后重试。\n"
            f"import error: {_cyclonedds_import_error}"
        )


MAX_MOTOR_NUM = 16
MAX_FAULT_NUM = 16


def _idl_kwargs_init(self, **kwargs):
    """让 IdlStruct 在不同 cyclonedds 版本下都能用 kwargs 构造。

    一些 cyclonedds 版本在反序列化时会直接调用 `Type(**valuedict)`；
    若 IdlStruct 子类没有 kwargs 构造，会报 `TypeError: Xxx() takes no arguments`。

    注意：不能通过把一个 mixin 放到继承列表首位来实现（会影响 cyclonedds 的 TypeObject 构建）。
    """
    for k, v in kwargs.items():
        setattr(self, k, v)


def _idl_make(cls, /, **kwargs):
    """兼容不同 cyclonedds 版本的 IdlStruct 构造方式。

    有些版本的 IdlStruct 不支持带参构造（kwargs），只能先无参创建再逐字段赋值。
    """
    try:
        return cls(**kwargs)
    except (TypeError, ValueError):
        # 某些版本的 IdlStruct 可能不允许无参构造，尝试使用 __new__ 和直接字段赋值
        try:
            obj = cls.__new__(cls)
            for k, v in kwargs.items():
                setattr(obj, k, v)
            return obj
        except Exception:
            # 如果 __new__ 也失败，尝试无参构造（某些版本可能支持）
            try:
                obj = cls()
                for k, v in kwargs.items():
                    setattr(obj, k, v)
                return obj
            except Exception as e:
                raise RuntimeError(f"无法创建 {cls.__name__} 实例: {e}") from e


def _try_make_typedef(alias_typename: str, base_type):
    """尽量把 IDL 中的 typedef 映射到 cyclonedds Python 的类型描述符。

    MotionCmd.idl / MotionObserved.idl 在 `uniubi::msg::dds_` 里用了：
      typedef uniubi::dds_::IMUState IMUState;
      typedef uniubi::dds_::MotorCtrl MotorCtrl;
      typedef uniubi::dds_::MotorObserved MotorObserved;

    如果 Python 端把字段直接写成 `uniubi::dds_::Xxx`，TypeIdentifier 可能与对端不同，导致匹配失败。
    """
    if types is None:  # pragma: no cover
        return base_type
    td = getattr(types, "typedef", None)
    if td is None:  # pragma: no cover
        return base_type

    # cyclonedds 不同版本 typedef 的调用签名可能不同，这里做多种尝试。
    candidates = [
        lambda: td(base_type, alias_typename),
        lambda: td(alias_typename, base_type),
        lambda: td(base_type, typename=alias_typename),
        lambda: td(base_type, name=alias_typename),
        lambda: td(typename=alias_typename, subtype=base_type),
        lambda: td(name=alias_typename, subtype=base_type),
        lambda: td(subtype=base_type, name=alias_typename),
        lambda: td(subtype=base_type, typename=alias_typename),
    ]
    for fn in candidates:
        try:
            out = fn()
        except TypeError:
            continue
        except Exception:
            continue
        # 过滤掉“把 alias 当成 subtype 字符串”的错误构造，否则 populate 会报无法解析的类型名。
        try:
            sub = getattr(out, "subtype", None)
            if isinstance(sub, str) and ("::" in sub or sub == alias_typename):
                continue
        except Exception:
            pass
        return out
    return base_type


if types is not None:

    class Quaternionf(IdlStruct, typename="uniubi::dds_::Quaternionf"):  # pragma: no cover
        error: types.int8
        w: types.float32
        x: types.float32
        y: types.float32
        z: types.float32
        __init__ = _idl_kwargs_init


    class Vector3f(IdlStruct, typename="uniubi::dds_::Vector3f"):  # pragma: no cover
        error: types.int8
        x: types.float32
        y: types.float32
        z: types.float32
        __init__ = _idl_kwargs_init


    class IMUState(IdlStruct, typename="uniubi::dds_::IMUState"):  # pragma: no cover
        temp: types.float32
        accel: Vector3f
        gyro: Vector3f
        mag: Vector3f
        euler: Vector3f
        quaternion: Quaternionf
        __init__ = _idl_kwargs_init


    class MotorHeader_(IdlStruct, typename="uniubi::dds_::MotorHeader"):  # pragma: no cover
        limbsNo: types.uint32
        jointNo: types.uint32
        __init__ = _idl_kwargs_init


    class MotorObserved(IdlStruct, typename="uniubi::dds_::MotorObserved"):  # pragma: no cover
        enable: types.uint8
        online: types.uint8
        error: types.uint8
        position: types.float32
        velocity: types.float32
        torque: types.float32
        temp: types.float32
        voltage: types.float32
        lossRate: types.float32
        maxTorque: types.float32
        header: MotorHeader_
        __init__ = _idl_kwargs_init


    class PowerObserved(IdlStruct, typename="uniubi::dds_::PowerObserved"):  # pragma: no cover
        power: types.float32
        health: types.float32
        temper: types.float32
        chargeCurrent: types.float32
        chargeVoltage: types.float32
        __init__ = _idl_kwargs_init


    class MotorCtrl_(IdlStruct, typename="uniubi::dds_::MotorCtrl"):  # pragma: no cover
        position: types.float32
        velocity: types.float32
        kpGain: types.float32
        kdGain: types.float32
        torque: types.float32
        header: MotorHeader_
        __init__ = _idl_kwargs_init

    # === msg 命名空间里的 typedef 别名（必须对齐，否则 type_id 会不一致） ===
    IMUStateMsg = _try_make_typedef("uniubi::msg::dds_::IMUState", IMUState)
    MotorCtrlMsg = _try_make_typedef("uniubi::msg::dds_::MotorCtrl", MotorCtrl_)
    MotorObservedMsg = _try_make_typedef("uniubi::msg::dds_::MotorObserved", MotorObserved)
    PowerObservedMsg = _try_make_typedef("uniubi::msg::dds_::PowerObserved", PowerObserved)

    def _usable_idl_type(t) -> bool:
        return hasattr(t, "__idl__") or hasattr(t, "__idl_typename__") or hasattr(t, "subtype")

    # 有些 cyclonedds 版本 typedef 返回的对象不是结构体类；若不可用就回退到 base（至少不崩溃）
    if not _usable_idl_type(IMUStateMsg):
        IMUStateMsg = IMUState
    if not _usable_idl_type(MotorCtrlMsg):
        MotorCtrlMsg = MotorCtrl_
    if not _usable_idl_type(MotorObservedMsg):
        MotorObservedMsg = MotorObserved
    if not _usable_idl_type(PowerObservedMsg):
        PowerObservedMsg = PowerObserved

    # cyclonedds 的类型归一化会把“未展开类型名”当作 module attribute 来 getattr；
    # 对端的 full typename 带 `::`，需要在本模块里显式注册，避免报：
    #   module ... has no attribute 'uniubi::msg::dds_::IMUState'
    _this_mod = sys.modules[__name__]
    try:
        setattr(_this_mod, "uniubi::msg::dds_::IMUState", IMUStateMsg)
        setattr(_this_mod, "uniubi::msg::dds_::MotorCtrl", MotorCtrlMsg)
        setattr(_this_mod, "uniubi::msg::dds_::MotorObserved", MotorObservedMsg)
        setattr(_this_mod, "uniubi::msg::dds_::PowerObserved", PowerObservedMsg)
    except Exception:  # pragma: no cover
        pass

    _MotorCtrlElem = MotorCtrlMsg
    _MotorObservedElem = MotorObservedMsg
    try:
        _ = types.array[_MotorCtrlElem, MAX_MOTOR_NUM]
    except Exception:  # pragma: no cover
        _MotorCtrlElem = MotorCtrl_
    try:
        _ = types.array[_MotorObservedElem, MAX_MOTOR_NUM]
    except Exception:  # pragma: no cover
        _MotorObservedElem = MotorObserved


    class MotionCtrl_(IdlStruct, typename="uniubi::msg::dds_::MotionCtrl_"):  # pragma: no cover
        motorNum: types.uint32
        timestamp: types.uint64
        motor: types.array[_MotorCtrlElem, MAX_MOTOR_NUM]
        __init__ = _idl_kwargs_init


    class MotionObserved_(IdlStruct, typename="uniubi::msg::dds_::MotionObserved_"):  # pragma: no cover
        imu: IMUStateMsg
        motorNum: types.int32
        timestamp: types.uint64
        motor: types.array[_MotorObservedElem, MAX_MOTOR_NUM]
        power: PowerObservedMsg
        __init__ = _idl_kwargs_init


    class RemoteControl_(IdlStruct, typename="uniubi::msg::dds_::RemoteControl_"):  # pragma: no cover
        controller: types.uint64
        timestamp: types.uint64
        back: types.uint8
        start: types.uint8
        lb: types.uint8
        rb: types.uint8
        f1: types.uint8
        f2: types.uint8
        a: types.uint8
        b: types.uint8
        x: types.uint8
        y: types.uint8
        up: types.uint8
        down: types.uint8
        left: types.uint8
        right: types.uint8
        ls: types.uint8
        rs: types.uint8
        stickLX: types.float32
        stickLY: types.float32
        stickRX: types.float32
        stickRY: types.float32
        triggerL: types.float32
        triggerR: types.float32
        __init__ = _idl_kwargs_init


    class MotionRecord_(IdlStruct, typename="uniubi::msg::dds_::MotionRecord_"):  # pragma: no cover
        action: types.int32
        observed: MotionObserved_
        control: RemoteControl_
        __init__ = _idl_kwargs_init


    class MotionFaultItem_(IdlStruct, typename="uniubi::msg::dds_::MotionFaultItem_"):  # pragma: no cover
        jointIndex: types.int32
        startDelayS: types.float32
        durationS: types.float32
        errorCode: types.uint8
        __init__ = _idl_kwargs_init


    class MotionFault_(IdlStruct, typename="uniubi::msg::dds_::MotionFault_"):  # pragma: no cover
        faultNum: types.uint32
        timestamp: types.uint64
        fault: types.array[MotionFaultItem_, MAX_FAULT_NUM]
        __init__ = _idl_kwargs_init

else:  # pragma: no cover
    # 让模块在缺少 cyclonedds 时也能被 import（运行时会在 _require_cyclonedds 里报错）
    Quaternionf = object
    Vector3f = object
    IMUState = object
    MotorHeader_ = object
    MotorObserved = object
    PowerObserved = object
    MotorCtrl_ = object
    MotionCtrl_ = object
    MotionObserved_ = object
    RemoteControl_ = object
    MotionRecord_ = object
    MotionFaultItem_ = object
    MotionFault_ = object


@dataclass(frozen=True)
class CycloneDdsConfig:
    domain_id: int
    topic_motion_observed: str
    topic_motion_control: str
    topic_motion_fault: str = "rt/motion/fault"
    topic_motion_record: str = "rt/motion/record"
    # per-topic QoS: observed -> BEST_EFFORT, control -> RELIABLE
    observed_reliability: str = "BEST_EFFORT"  # BEST_EFFORT | RELIABLE
    observed_history: str = "KEEP_LAST"  # KEEP_LAST | KEEP_ALL
    observed_depth: int = 10
    observed_reliable_max_blocking_time_ns: int = 0
    observed_data_representation_cdrv0: bool = True
    observed_data_representation_xcdrv2: bool = True
    control_reliability: str = "RELIABLE"  # BEST_EFFORT | RELIABLE
    control_history: str = "KEEP_LAST"  # KEEP_LAST | KEEP_ALL
    control_depth: int = 10
    control_reliable_max_blocking_time_ns: int = 0
    control_data_representation_cdrv0: bool = True
    control_data_representation_xcdrv2: bool = True
    fault_reliability: str = "RELIABLE"  # BEST_EFFORT | RELIABLE
    fault_history: str = "KEEP_LAST"  # KEEP_LAST | KEEP_ALL
    fault_depth: int = 10
    fault_reliable_max_blocking_time_ns: int = 0
    fault_data_representation_cdrv0: bool = True
    fault_data_representation_xcdrv2: bool = True
    debug: bool = False
    debug_prefix: str = "DDS"


def _build_qos(
    reliability: str,
    history: str,
    depth: int,
    reliable_max_blocking_time_ns: int,
    data_representation_cdrv0: bool,
    data_representation_xcdrv2: bool,
):
    if Qos is None or Policy is None:  # pragma: no cover
        return None

    reliability = str(reliability).upper()
    history = str(history).upper()

    policies = []
    if reliability == "BEST_EFFORT":
        policies.append(Policy.Reliability.BestEffort)
    elif reliability == "RELIABLE":
        # cyclonedds 里 Reliable 需要 max_blocking_time（纳秒）
        policies.append(Policy.Reliability.Reliable(int(reliable_max_blocking_time_ns)))
    else:
        raise ValueError(f"未知 reliability: {reliability}")

    if history == "KEEP_LAST":
        policies.append(Policy.History.KeepLast(int(depth)))
    elif history == "KEEP_ALL":
        policies.append(Policy.History.KeepAll)
    else:
        raise ValueError(f"未知 history: {history}")

    # DataRepresentation：显式声明，避免与对端默认值不一致导致 incompatible_qos（常见 last_policy_id=24）。
    try:
        policies.append(
            Policy.DataRepresentation(
                use_cdrv0_representation=bool(data_representation_cdrv0),
                use_xcdrv2_representation=bool(data_representation_xcdrv2),
            )
        )
    except Exception:
        # 某些老版本 cyclonedds 可能没有该 policy，忽略即可
        pass

    return Qos(*policies)


def _make_writer(participant, topic, qos):
    if qos is None:
        return DataWriter(participant, topic)
    try:
        return DataWriter(participant, topic, qos=qos)
    except TypeError:  # pragma: no cover
        try:
            return DataWriter(participant, topic, qos)
        except TypeError:
            return DataWriter(participant, topic)


def _make_reader(participant, topic, qos):
    if qos is None:
        return DataReader(participant, topic)
    try:
        return DataReader(participant, topic, qos=qos)
    except TypeError:  # pragma: no cover
        try:
            return DataReader(participant, topic, qos)
        except TypeError:
            return DataReader(participant, topic)


def _make_writer_with_listener(participant, topic, qos, listener):
    if listener is None:
        return _make_writer(participant, topic, qos)
    try:
        return DataWriter(participant, topic, qos=qos, listener=listener)
    except TypeError:  # pragma: no cover
        try:
            return DataWriter(participant, topic, qos, listener)
        except TypeError:
            return _make_writer(participant, topic, qos)


def _make_reader_with_listener(participant, topic, qos, listener):
    if listener is None:
        return _make_reader(participant, topic, qos)
    try:
        return DataReader(participant, topic, qos=qos, listener=listener)
    except TypeError:  # pragma: no cover
        try:
            return DataReader(participant, topic, qos, listener)
        except TypeError:
            return _make_reader(participant, topic, qos)


class CycloneDdsTransport(MotionTransport):
    def __init__(self, cfg: CycloneDdsConfig, motor_headers: Sequence[MotorHeader]):
        _require_cyclonedds()
        self._next_invalid_ctrl_sample_log_t = 0.0
        self._headers = tuple(motor_headers)
        self._zero_vec3 = np.zeros(3, dtype=np.float32)
        self._participant = DomainParticipant(cfg.domain_id)

        qos_observed = _build_qos(
            cfg.observed_reliability,
            cfg.observed_history,
            cfg.observed_depth,
            cfg.observed_reliable_max_blocking_time_ns,
            cfg.observed_data_representation_cdrv0,
            cfg.observed_data_representation_xcdrv2,
        )
        qos_control = _build_qos(
            cfg.control_reliability,
            cfg.control_history,
            cfg.control_depth,
            cfg.control_reliable_max_blocking_time_ns,
            cfg.control_data_representation_cdrv0,
            cfg.control_data_representation_xcdrv2,
        )
        qos_fault = _build_qos(
            cfg.fault_reliability,
            cfg.fault_history,
            cfg.fault_depth,
            cfg.fault_reliable_max_blocking_time_ns,
            cfg.fault_data_representation_cdrv0,
            cfg.fault_data_representation_xcdrv2,
        )
        # 注意：对端报 `last_policy_id=24`（常见是 DataRepresentation）时，
        # 需要把 DataRepresentation 等 topic-level QoS 放在 Topic 上（不仅仅是 reader/writer）。
        self._topic_obs = Topic(self._participant, cfg.topic_motion_observed, MotionObserved_, qos=qos_observed)
        self._topic_ctrl = Topic(self._participant, cfg.topic_motion_control, MotionCtrl_, qos=qos_control)
        self._topic_fault = Topic(self._participant, cfg.topic_motion_fault, MotionFault_, qos=qos_fault)
        self._topic_record = None
        self._writer_record = None
        if cfg.topic_motion_record:
            self._topic_record = Topic(self._participant, cfg.topic_motion_record, MotionRecord_, qos=qos_observed)

        self._matched_readers_for_obs_writer = 0
        self._matched_writers_for_ctrl_reader = 0

        pub_listener = None
        sub_listener = None
        if getattr(cfg, "debug", False) and Listener is not None:
            prefix = getattr(cfg, "debug_prefix", "DDS")

            def on_pub_matched(writer, status):
                self._matched_readers_for_obs_writer = int(status.current_count)
                print(
                    f"[{prefix}] publication_matched topic={cfg.topic_motion_observed} "
                    f"type={MotionObserved_.__idl_typename__} current={int(status.current_count)} "
                    f"change={int(status.current_count_change)} total={int(status.total_count)}"
                )

            def on_sub_matched(reader, status):
                self._matched_writers_for_ctrl_reader = int(status.current_count)
                print(
                    f"[{prefix}] subscription_matched topic={cfg.topic_motion_control} "
                    f"type={MotionCtrl_.__idl_typename__} current={int(status.current_count)} "
                    f"change={int(status.current_count_change)} total={int(status.total_count)}"
                )

            def on_offered_incompatible_qos(writer, status):
                print(
                    f"[{prefix}] offered_incompatible_qos topic={cfg.topic_motion_observed} "
                    f"total={int(status.total_count)} last_policy_id={int(status.last_policy_id)}"
                )

            def on_requested_incompatible_qos(reader, status):
                print(
                    f"[{prefix}] requested_incompatible_qos topic={cfg.topic_motion_control} "
                    f"total={int(status.total_count)} last_policy_id={int(status.last_policy_id)}"
                )

            def on_fault_sub_matched(reader, status):
                print(
                    f"[{prefix}] subscription_matched topic={cfg.topic_motion_fault} "
                    f"type={MotionFault_.__idl_typename__} current={int(status.current_count)} "
                    f"change={int(status.current_count_change)} total={int(status.total_count)}"
                )

            def on_fault_requested_incompatible_qos(reader, status):
                print(
                    f"[{prefix}] requested_incompatible_qos topic={cfg.topic_motion_fault} "
                    f"total={int(status.total_count)} last_policy_id={int(status.last_policy_id)}"
                )

            pub_listener = Listener(
                on_publication_matched=on_pub_matched,
                on_offered_incompatible_qos=on_offered_incompatible_qos,
            )
            sub_listener = Listener(
                on_subscription_matched=on_sub_matched,
                on_requested_incompatible_qos=on_requested_incompatible_qos,
            )
            fault_listener = Listener(
                on_subscription_matched=on_fault_sub_matched,
                on_requested_incompatible_qos=on_fault_requested_incompatible_qos,
            )
        else:
            fault_listener = None

        self._writer_obs = _make_writer_with_listener(self._participant, self._topic_obs, qos_observed, pub_listener)
        if self._topic_record is not None:
            self._writer_record = _make_writer(self._participant, self._topic_record, qos_observed)

        self._reader_ctrl = _make_reader_with_listener(self._participant, self._topic_ctrl, qos_control, sub_listener)
        self._reader_fault = _make_reader_with_listener(self._participant, self._topic_fault, qos_fault, fault_listener)
        self._motor_item_cls = self._resolve_motor_observed_cls()
        self._imu_cls = self._resolve_imu_cls()
        self._obs_headers = self._build_observed_headers()
        self._obs_motors = self._build_observed_motors()
        self._obs_accel = _idl_make(Vector3f, error=0, x=0.0, y=0.0, z=0.0)
        self._obs_gyro = _idl_make(Vector3f, error=0, x=0.0, y=0.0, z=0.0)
        self._obs_mag = _idl_make(Vector3f, error=0, x=0.0, y=0.0, z=0.0)
        self._obs_euler = _idl_make(Vector3f, error=0, x=0.0, y=0.0, z=0.0)
        self._obs_quaternion = _idl_make(Quaternionf, error=0, w=1.0, x=0.0, y=0.0, z=0.0)
        self._obs_imu = _idl_make(
            self._imu_cls,
            temp=0.0,
            accel=self._obs_accel,
            gyro=self._obs_gyro,
            mag=self._obs_mag,
            euler=self._obs_euler,
            quaternion=self._obs_quaternion,
        )
        self._obs_power = _idl_make(
            PowerObserved,
            power=0.0,
            health=0.0,
            temper=0.0,
            chargeCurrent=0.0,
            chargeVoltage=0.0,
        )
        self._obs_msg = _idl_make(
            MotionObserved_,
            imu=self._obs_imu,
            motorNum=0,
            timestamp=0,
            motor=self._obs_motors,
            power=self._obs_power,
        )
        self._record_control_msg = _idl_make(
            RemoteControl_,
            controller=0,
            timestamp=0,
            back=0,
            start=0,
            lb=0,
            rb=0,
            f1=0,
            f2=0,
            a=0,
            b=0,
            x=0,
            y=0,
            up=0,
            down=0,
            left=0,
            right=0,
            ls=0,
            rs=0,
            stickLX=0.0,
            stickLY=0.0,
            stickRX=0.0,
            stickRY=0.0,
            triggerL=0.0,
            triggerR=0.0,
        )
        self._record_msg = (
            _idl_make(MotionRecord_, action=0, observed=self._obs_msg, control=self._record_control_msg)
            if self._writer_record is not None
            else None
        )
        self._fault_item_default = _idl_make(
            MotionFaultItem_,
            jointIndex=-1,
            startDelayS=0.0,
            durationS=0.0,
            errorCode=0,
        )
        self._last_obs_motor_num = 0

        # Builtin endpoint discovery: 用于打印“对端”的 QoS（publication/subscription builtin topics）
        self._builtin_pub_reader = None
        self._builtin_sub_reader = None
        if getattr(cfg, "debug", False) and BuiltinDataReader is not None:
            try:
                self._builtin_pub_reader = BuiltinDataReader(self._participant, BuiltinTopicDcpsPublication)
                self._builtin_sub_reader = BuiltinDataReader(self._participant, BuiltinTopicDcpsSubscription)
            except Exception:
                self._builtin_pub_reader = None
                self._builtin_sub_reader = None

        # 记录 DDS 层最终使用的 type name（用于确认是否与对端一致）
        try:
            self._topic_obs_typename = str(self._topic_obs.typename)
        except Exception:  # pragma: no cover
            self._topic_obs_typename = MotionObserved_.__idl_typename__
        try:
            self._topic_ctrl_typename = str(self._topic_ctrl.typename)
        except Exception:  # pragma: no cover
            self._topic_ctrl_typename = MotionCtrl_.__idl_typename__
        try:
            self._topic_fault_typename = str(self._topic_fault.typename)
        except Exception:  # pragma: no cover
            self._topic_fault_typename = MotionFault_.__idl_typename__

        # 本端 type identifier（XTypes），用于定位“类型不一致/不可赋值”导致的 incompatible_qos
        try:
            MotionObserved_.__idl__.populate()
            MotionObserved_.__idl__.fill_type_data()
            self._local_obs_type_id = str(MotionObserved_.__idl__.get_type_id())
        except Exception:  # pragma: no cover
            self._local_obs_type_id = "<unknown>"
        try:
            MotionCtrl_.__idl__.populate()
            MotionCtrl_.__idl__.fill_type_data()
            self._local_ctrl_type_id = str(MotionCtrl_.__idl__.get_type_id())
        except Exception:  # pragma: no cover
            self._local_ctrl_type_id = "<unknown>"
        try:
            MotionFault_.__idl__.populate()
            MotionFault_.__idl__.fill_type_data()
            self._local_fault_type_id = str(MotionFault_.__idl__.get_type_id())
        except Exception:  # pragma: no cover
            self._local_fault_type_id = "<unknown>"

        if getattr(cfg, "debug", False):
            prefix = getattr(cfg, "debug_prefix", "DDS")
            try:
                imu_alias = globals().get("IMUStateMsg", IMUState)
                ctrl_alias = globals().get("MotorCtrlMsg", MotorCtrl_)
                obs_alias = globals().get("MotorObservedMsg", MotorObserved)
                power_alias = globals().get("PowerObservedMsg", PowerObserved)

                def _td_desc(t) -> str:
                    return getattr(t, "__idl_typename__", None) or str(t)

                print(
                    f"[{prefix}] typedef_map "
                    f"IMUState={_td_desc(imu_alias)} MotorCtrl={_td_desc(ctrl_alias)} "
                    f"MotorObserved={_td_desc(obs_alias)} PowerObserved={_td_desc(power_alias)}"
                )
            except Exception:  # pragma: no cover
                pass
            print(
                f"[{prefix}] topic_created name={cfg.topic_motion_observed} typename={self._topic_obs_typename} "
                f"qos={cfg.observed_reliability}/{cfg.observed_history}(depth={cfg.observed_depth}) "
                f"qos_policies={qos_observed} type_id={self._local_obs_type_id}"
            )
            print(
                f"[{prefix}] topic_created name={cfg.topic_motion_control} typename={self._topic_ctrl_typename} "
                f"qos={cfg.control_reliability}/{cfg.control_history}(depth={cfg.control_depth}) "
                f"qos_policies={qos_control} type_id={self._local_ctrl_type_id}"
            )
            print(
                f"[{prefix}] topic_created name={cfg.topic_motion_fault} typename={self._topic_fault_typename} "
                f"qos={cfg.fault_reliability}/{cfg.fault_history}(depth={cfg.fault_depth}) "
                f"qos_policies={qos_fault} type_id={self._local_fault_type_id}"
            )

    def _resolve_motor_observed_cls(self):
        motor_item_cls = MotorObserved
        try:
            motor_elem = globals().get("_MotorObservedElem")
            if isinstance(motor_elem, type):
                motor_item_cls = motor_elem
        except Exception:  # pragma: no cover
            motor_item_cls = MotorObserved
        return motor_item_cls

    def _resolve_imu_cls(self):
        imu_cls = IMUState
        try:
            imu_alias = globals().get("IMUStateMsg")
            if isinstance(imu_alias, type):
                imu_cls = imu_alias
        except Exception:  # pragma: no cover
            imu_cls = IMUState
        return imu_cls

    def _build_observed_headers(self):
        headers = []
        for i in range(MAX_MOTOR_NUM):
            if i < len(self._headers):
                h = self._headers[i]
                limbs_no = int(h.limbs_no)
                joint_no = int(h.joint_no)
            else:
                limbs_no = 0
                joint_no = 0
            headers.append(_idl_make(MotorHeader_, limbsNo=limbs_no, jointNo=joint_no))
        return tuple(headers)

    def _build_observed_motors(self):
        motors = []
        for i in range(MAX_MOTOR_NUM):
            motors.append(
                _idl_make(
                    self._motor_item_cls,
                    enable=0,
                    online=0,
                    error=0,
                    position=0.0,
                    velocity=0.0,
                    torque=0.0,
                    temp=0.0,
                    voltage=0.0,
                    lossRate=0.0,
                    maxTorque=0.0,
                    header=self._obs_headers[i],
                )
            )
        return motors

    def debug_match_counts(self) -> Tuple[int, int]:
        """返回 (obs_writer_matched_readers, ctrl_reader_matched_writers)。"""
        return int(self._matched_readers_for_obs_writer), int(self._matched_writers_for_ctrl_reader)

    def debug_topic_typenames(self) -> Tuple[str, str]:
        """返回 (observed_topic_typename, control_topic_typename)。"""
        return str(self._topic_obs_typename), str(self._topic_ctrl_typename)

    def debug_local_type_ids(self) -> Tuple[str, str]:
        """返回 (observed_type_id, control_type_id) 的字符串表示。"""
        return str(self._local_obs_type_id), str(self._local_ctrl_type_id)

    def debug_peer_qos_summary(self) -> Dict[str, List[str]]:
        """从 builtin topics 中抓取对端 endpoint 的 QoS（用于定位 incompatible_qos）。

        返回：
        - key: "peer_subscriptions_for_observed" / "peer_publications_for_control"
        - value: 多行字符串（包含 participant_key 与 QoS 摘要）
        """

        def _qos_summary(qos_obj) -> str:
            if qos_obj is None or Policy is None:
                return "<unknown>"
            parts = []
            try:
                if Policy.Reliability in qos_obj:
                    parts.append(str(qos_obj[Policy.Reliability]))
            except Exception:
                pass
            try:
                if Policy.History in qos_obj:
                    parts.append(str(qos_obj[Policy.History]))
            except Exception:
                pass
            try:
                if Policy.DataRepresentation in qos_obj:
                    parts.append(str(qos_obj[Policy.DataRepresentation]))
            except Exception:
                pass
            if parts:
                return ", ".join(parts)
            try:
                return str(qos_obj)
            except Exception:
                return "<unprintable>"

        out: Dict[str, List[str]] = {
            "peer_subscriptions_for_observed": [],
            "peer_publications_for_control": [],
        }

        # 没有 builtin reader 就无法枚举对端 QoS
        if self._builtin_pub_reader is None or self._builtin_sub_reader is None:
            return out

        # 读取 endpoint 列表（非阻塞，尽量多读一点）
        try:
            pubs = self._builtin_pub_reader.read(64)
        except Exception:
            pubs = []
        try:
            subs = self._builtin_sub_reader.read(64)
        except Exception:
            subs = []

        # 对端在 observed topic 上是“订阅者”
        for ep in subs:
            try:
                if ep.topic_name == self._topic_obs.name and ep.type_name == self._topic_obs_typename:
                    type_id = getattr(ep, "type_id", None)
                    type_id_s = "<none>" if type_id is None else str(type_id)
                    out["peer_subscriptions_for_observed"].append(
                        f"participant={ep.participant_key} qos={_qos_summary(ep.qos)} type_id={type_id_s}"
                    )
            except Exception:
                continue

        # 对端在 control topic 上是“发布者”
        for ep in pubs:
            try:
                if ep.topic_name == self._topic_ctrl.name and ep.type_name == self._topic_ctrl_typename:
                    type_id = getattr(ep, "type_id", None)
                    type_id_s = "<none>" if type_id is None else str(type_id)
                    out["peer_publications_for_control"].append(
                        f"participant={ep.participant_key} qos={_qos_summary(ep.qos)} type_id={type_id_s}"
                    )
            except Exception:
                continue

        return out

    def publish_observed(self, msg: MotionObserved) -> None:
        motor_num = int(msg.motor_pos.shape[0])
        if motor_num > MAX_MOTOR_NUM:
            raise ValueError(f"motor_num({motor_num}) > MAX_MOTOR_NUM({MAX_MOTOR_NUM})")

        motor_enable = None if msg.motor_enable is None else np.asarray(msg.motor_enable, dtype=np.uint8).reshape(-1)
        motor_online = None if msg.motor_online is None else np.asarray(msg.motor_online, dtype=np.uint8).reshape(-1)
        motor_error = None if msg.motor_error is None else np.asarray(msg.motor_error, dtype=np.uint8).reshape(-1)
        motor_pos = np.asarray(msg.motor_pos, dtype=np.float32)
        motor_vel = np.asarray(msg.motor_vel, dtype=np.float32)
        motor_tau = np.asarray(msg.motor_tau, dtype=np.float32)
        quat = np.asarray(msg.quat_wxyz, dtype=np.float32)
        gyro = np.asarray(msg.gyro_xyz, dtype=np.float32)
        accel = np.asarray(msg.accel_xyz, dtype=np.float32)

        self._obs_imu.temp = float(msg.imu_temp)
        self._obs_accel.x = float(accel[0])
        self._obs_accel.y = float(accel[1])
        self._obs_accel.z = float(accel[2])
        self._obs_gyro.x = float(gyro[0])
        self._obs_gyro.y = float(gyro[1])
        self._obs_gyro.z = float(gyro[2])
        self._obs_quaternion.w = float(quat[0])
        self._obs_quaternion.x = float(quat[1])
        self._obs_quaternion.y = float(quat[2])
        self._obs_quaternion.z = float(quat[3])
        self._obs_power.power = float(msg.power.power)
        self._obs_power.health = float(msg.power.health)
        self._obs_power.temper = float(msg.power.temper)
        self._obs_power.chargeCurrent = float(msg.power.charge_current)
        self._obs_power.chargeVoltage = float(msg.power.charge_voltage)

        for i in range(motor_num):
            motor = self._obs_motors[i]
            motor.enable = int(motor_enable[i]) if (motor_enable is not None and i < motor_enable.shape[0]) else 1
            motor.online = int(motor_online[i]) if (motor_online is not None and i < motor_online.shape[0]) else 1
            motor.error = int(motor_error[i]) if (motor_error is not None and i < motor_error.shape[0]) else 0
            motor.position = float(motor_pos[i])
            motor.velocity = float(motor_vel[i])
            motor.torque = float(motor_tau[i])
            motor.temp = 0.0
            motor.voltage = 0.0
            motor.lossRate = 0.0
            motor.maxTorque = 0.0

        for i in range(motor_num, self._last_obs_motor_num):
            motor = self._obs_motors[i]
            motor.enable = 0
            motor.online = 0
            motor.error = 0
            motor.position = 0.0
            motor.velocity = 0.0
            motor.torque = 0.0
            motor.temp = 0.0
            motor.voltage = 0.0
            motor.lossRate = 0.0
            motor.maxTorque = 0.0

        self._last_obs_motor_num = motor_num
        self._obs_msg.motorNum = int(motor_num)
        self._obs_msg.timestamp = int(msg.timestamp_us)
        self._writer_obs.write(self._obs_msg)
        if self._writer_record is not None and self._record_msg is not None:
            self._writer_record.write(self._record_msg)

    def try_recv_control(self, timeout_s: float) -> Optional[MotionCtrl]:
        deadline = time.time() + max(0.0, timeout_s)
        while True:
            samples = self._reader_ctrl.take(1)
            if samples:
                data = samples[0]
                # 避免主板应用退出引起的崩溃
                if (data is None) or (not hasattr(data, "motorNum")):
                    now = time.time()
                    if now >= self._next_invalid_ctrl_sample_log_t:
                        print(
                            "[DDS] WARN skip invalid control sample "
                            f"type={type(data).__name__}"
                        )
                        self._next_invalid_ctrl_sample_log_t = now + 1.0
                    if time.time() >= deadline:
                        return None
                    continue
                motor_num = int(data.motorNum)
                motor_num = min(motor_num, MAX_MOTOR_NUM)

                pos = np.zeros(motor_num, dtype=np.float32)
                vel = np.zeros(motor_num, dtype=np.float32)
                kp = np.zeros(motor_num, dtype=np.float32)
                kd = np.zeros(motor_num, dtype=np.float32)
                tau = np.zeros(motor_num, dtype=np.float32)
                headers: list[MotorHeader] = []

                for i in range(motor_num):
                    m = data.motor[i]
                    # 注意：部分对端实现可能不会填充 velocity 字段（或未初始化），会出现 NaN。
                    # 这里做一次健壮性处理，避免上层把 NaN 继续用于控制/观测。
                    pos_i = float(m.position)
                    vel_i = float(m.velocity)
                    kp_i = float(m.kpGain)
                    kd_i = float(m.kdGain)
                    tau_i = float(m.torque)

                    pos[i] = pos_i if np.isfinite(pos_i) else 0.0
                    vel[i] = vel_i if np.isfinite(vel_i) else 0.0
                    kp[i] = kp_i if np.isfinite(kp_i) else 0.0
                    kd[i] = kd_i if np.isfinite(kd_i) else 0.0
                    tau[i] = tau_i if np.isfinite(tau_i) else 0.0
                    headers.append(MotorHeader(limbs_no=int(m.header.limbsNo), joint_no=int(m.header.jointNo)))

                return MotionCtrl(
                    timestamp_us=int(data.timestamp),
                    motor_pos=pos,
                    motor_vel=vel,
                    kp=kp,
                    kd=kd,
                    tau_ff=tau,
                    motor_headers=tuple(headers),
                )

            if time.time() >= deadline:
                return None
            time.sleep(0.0005)

    def try_recv_faults(self, timeout_s: float) -> list[MotionFault]:
        deadline = time.time() + max(0.0, timeout_s)
        out: list[MotionFault] = []
        while True:
            samples = self._reader_fault.take(1)
            if samples:
                data = samples[0]
                if (data is None) or (not hasattr(data, "faultNum")):
                    if time.time() >= deadline:
                        return out
                    continue

                fault_num = min(int(data.faultNum), MAX_FAULT_NUM)
                items: list[MotionFaultItem] = []
                for i in range(fault_num):
                    item = data.fault[i]
                    items.append(
                        MotionFaultItem(
                            joint_index=int(item.jointIndex),
                            start_delay_s=float(item.startDelayS),
                            duration_s=float(item.durationS),
                            error_code=int(item.errorCode),
                        )
                    )
                out.append(MotionFault(timestamp_us=int(data.timestamp), items=tuple(items)))
                continue

            if out or time.time() >= deadline:
                return out
            time.sleep(0.0005)
