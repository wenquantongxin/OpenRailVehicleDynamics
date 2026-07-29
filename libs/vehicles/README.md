# libs/vehicles

**职责**：整车组装模板。`gz18/` 为首车型；模板化以便日后扩展 SH17 等刚性轮对车型。

**GZ18 拓扑**（第一手实测）：
- 用户刚体 17（车体 1 + 构架 2 + DUM 2 + 轮对 4 + 轴箱载体 8）；`num_bodies()=18` 含 world。
- 关节：7 QuaternionFloating + 8 Revolute + 2 Weld；作动器 0。
- `nq=57`、`nv=50`、基础 `N=107`。

**里程碑**：M2（组装）→ M3（快照/CLI）。

**必须复现的陷阱**：**深度优先坐标分配使浮动关节与其轴箱转动关节交错**
（carbody 0/0、frame_front 7/6、wheelset_ff 21/18、rev_ff 28,29/24,25…），
不是"浮动排完再排转动"——任何自写索引都不得按后者直觉走（abstol 错位即前车之鉴）。
状态布局 `q=[qw,qx,qy,qz,px,py,pz]`、`v=[wx,wy,wz,vx,vy,vz]`（角速度在前）。详见设计基线 §3.1、§10.1（A3）。

`include/orvd/vehicles/` = public 头；`src/gz18/` = GZ18 组装实现。
