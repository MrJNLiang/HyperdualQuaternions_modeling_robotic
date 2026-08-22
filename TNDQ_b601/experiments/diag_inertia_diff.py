"""逐连杆参数对比：pinocchio reduced 模型 vs 提取回填表（q=0，DH 系）。

定位对账残差来源：若某连杆质量/质心/惯量差异 ~1e-6 相对量级，
则提取链路存在精度损失；若全部 ~1e-15，则差异在 RNEA 实现层。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import B601_URDF, B601_DH_TABLE, B601_BASE_PREFIX
from config.b601_dynamics import (
    B601_LINK_MASS, B601_LINK_COM, B601_LINK_INERTIA, _dh_transform)

import pinocchio as pin  # noqa: E402

model = pin.buildModelFromUrdf(B601_URDF)
jids = [model.getJointId(nm) for nm in ("gripper_joint1", "gripper_joint2")]
red = pin.buildReducedModel(model, jids, pin.neutral(model))
data = red.createData()

pin.forwardKinematics(red, data, pin.neutral(red))

# 我的 DH 帧世界位姿（base_link 系，q=0）
T = np.eye(4)
T[:3, 3] = B601_BASE_PREFIX
dh_world = []
for row in B601_DH_TABLE:
    a, al, d, off, _ = row
    T = T @ _dh_transform(a, al, d, off)
    dh_world.append(T.copy())

print("逐连杆对比（pin reduced 关节 i 惯性 -> DH 连杆 i 系）：")
for i in range(1, 7):
    oMi = data.oMi[i]
    R_pin, p_pin = oMi.rotation, np.array(oMi.translation)
    inertia = red.inertias[i]
    m_pin = inertia.mass
    c_world = R_pin @ np.array(inertia.lever) + p_pin
    I_world = R_pin @ np.asarray(inertia.inertia) @ R_pin.T

    Tdh = dh_world[i - 1]
    R_dh, p_dh = Tdh[:3, :3], Tdh[:3, 3]
    c_pin_dh = R_dh.T @ (c_world - p_dh)
    I_pin_dh = R_dh.T @ I_world @ R_dh

    m_mine, c_mine, I_mine = (B601_LINK_MASS[i - 1],
                               B601_LINK_COM[i - 1],
                               B601_LINK_INERTIA[i - 1])
    dm = m_mine - m_pin
    dc = c_mine - c_pin_dh
    dI = np.abs(I_mine - I_pin_dh).max()
    print(f"link{i}: m_pin={m_pin:.9f} m_mine={m_mine:.9f} dm={dm:+.2e}")
    print(f"        c_pin={np.array2string(c_pin_dh, precision=9)}")
    print(f"        c_mine={np.array2string(c_mine, precision=9)}")
    print(f"        dc={np.array2string(dc, precision=3)}  dI_max={dI:.3e}")
    # 相对差异（排除极小惯量分量的干扰，按 Frobenius 范数）
    rel_I = np.linalg.norm(I_mine - I_pin_dh) / np.linalg.norm(I_pin_dh)
    print(f"        质心偏差范数={np.linalg.norm(dc):.3e} m, "
          f"惯量相对差={rel_I:.3e}")
