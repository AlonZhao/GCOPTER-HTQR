# GCOPTER 代码库完整中文文档

> 本文档由对 16 个核心模块的系统分析整理而成，配合博士论文《A Geometrical Approach to Multicopter Motion Planning》阅读。

---

## 1. 项目概述 (Project Overview)

### 1.1 GCOPTER 是什么

GCOPTER (Geometrically COnstrained Polynomial TrajEctory geneRator) 是浙江大学 FAST Lab 开发的一款高效、通用的多旋翼无人机轨迹优化器。它基于一种新颖的稀疏轨迹表示方法 **MINCO** (Minimum Control) 构建，能够实时生成高质量、动力学可行的飞行轨迹。相关工作发表于 IEEE Transactions on Robotics (T-RO, 2022)。

其核心能力是：在用户定义的状态-输入约束（速度、机体角速度、倾角、推力等）下，为包含非线性阻力效应的飞行器动力学生成时空联合最优的轨迹。

### 1.2 核心特性和优势

- **稀疏轨迹表示 (MINCO)**：实现几何（空间路径）与时间（分段时长）的联合优化，使优化效率足以支撑实时走廊生成与全局轨迹生成。
- **微分平坦性支持**：在平坦输出空间中优化，净推力、倾角、机体角速度等量可直接获取并施加约束，且支持非线性阻力建模。
- **安全飞行走廊 (SFC)**：内置快速走廊生成算法 FIRI（Fast Iterative Region Inflation），将自由空间分解为一系列重叠的凸多面体。
- **多阶支持**：支持 s=2（最小加速度）、s=3（最小 Jerk）、s=4（最小 Snap）的均匀与非均匀 MINCO。
- **约束消除技术**：通过光滑重参数化将正时间约束和走廊隶属约束转化为无约束问题，整个优化归约为单次光滑无约束 L-BFGS 求解。

### 1.3 应用场景

GCOPTER/MINCO 已成为众多前沿项目的底层引擎：

- 鲁棒实时 SE(3) 全身规划（被 IEEE Spectrum 报道）
- 高速 FPV 第一人称飞行规划
- 多机集群规划（EGO-Planner-v2）
- 长距离无人机竞速（Fast-Racing, IEEE RA-L）
- 注视点遥操作规划（GPA-Teleoperation, IEEE RA-L）
- 编队保持规划（Swarm-Formation, IEEE ICRA）
- 可见性感知的空中跟踪、含非线性阻力的规划等

---

## 2. 核心概念 (Core Concepts)

### 2.1 MINCO 轨迹表示

**MINCO** 是一种稀疏的轨迹参数化方法，通过最小化高阶导数（控制量）生成平滑的分段多项式轨迹。三种变体对应不同平滑阶数 `s`：

| 变体 | 最小化目标 | 多项式阶数 | 每段系数数 |
|------|-----------|-----------|-----------|
| MINCO_S2NU | 加速度 (s=2) | 3 次 | 4 |
| MINCO_S3NU | Jerk (s=3) | 5 次 | 6 |
| MINCO_S4NU | Snap (s=4) | 7 次 | 8 |

"NU" 表示各段时间分配非均匀 (Non-Uniform)。

**稀疏性的关键**：轨迹仅由以下量表示：
- 中间路径点（仅各段交界处的位置）
- 各段的时间时长
- 起止边界条件（位置、速度、加速度、jerk）

完整的多项式系数通过求解一个**带状线性系统** `Ax = b` 得到，无需在优化过程中显式存储所有系数。约束矩阵 A 的带宽为 2s，因为连续性约束只耦合相邻段，从而实现 O(N) 求解复杂度（对比稠密系统的 O(N³)）。

**优化问题形式**：
```
minimize:  ∫ ||p^(s)(t)||² dt        (s 阶导数平方的积分)
subject to:
  - 边界条件:   p(0), p'(0), ..., p^(s-1)(0) 及末端对应量
  - 路径点约束: p_i(T_i) = waypoint_i
  - 连续性:     p_i^(k)(T_i) = p_{i+1}^(k)(0),  k = 0,...,s
```

### 2.2 几何约束优化（约束消除）

GCOPTER 的精髓在于**用光滑重参数化消除约束**，而非使用硬约束：

- **正时间约束**：时间变量 `tau → T` 通过光滑双射 `forwardT` 保证 `T > 0`（tau>0 时用二次式，tau≤0 时用倒数式）。
- **走廊隶属约束**：每个内部路径点用其所属多面体顶点的"平方归一化凸组合"表示（`q = xi.normalized()`, `P = V_rest·(q∘q) + V0`），从构造上保证点必在走廊内。
- **单纯形约束**：`normRetrictionLayer` 用三次软惩罚（关于 `‖q‖²−1`）保持凸组合权重落在单纯形上。

如此，整个问题归约为单次光滑无约束 L-BFGS 求解；动力学限制则作为可调的软惩罚处理。

### 2.3 微分平坦性与非线性阻力

**微分平坦性**让系统状态与控制输入能用平坦输出及其导数代数地表示。

- **平坦输出**：位置 (p) 和偏航角 (ψ)
- **正向映射** (`forward`)：平坦输出 → 完整状态（推力、四元数姿态、机体角速度）
- **反向映射** (`backward`)：反向模式自动微分，将代价/约束对完整状态的梯度回传到平坦输出

**三种阻力系数建模**：
- 水平阻力 (dh)：作用于水平速度分量
- 垂直阻力 (dv)：作用于垂直速度分量
- 寄生阻力 (cp)：与速度大小成正比

阻力公式：
```
cp_term = sqrt(v0² + v1² + v2² + veps)   // veps 防止零速奇异
w = (1 + cp · cp_term) · v
zu = a + (dh/mass) · w + [0, 0, grav]    // 推力方向向量
z  = zu / ||zu||                          // 机体 z 轴
f  = m·a + dv·w + [0, 0, m·grav]
thr = z · f                               // 推力大小
```

**约束的隐式编码**：倾角通过 `z2 = cos(tilt)` 约束（`tilt_den = sqrt(2(1+z2))` 在 180° 倾角处奇异）；机体角速度通过 `omg_den = z2 + 1` 耦合倾角与角速度约束。

### 2.4 走廊生成

自由空间被分解为一系列**重叠的凸多面体**，构成安全飞行走廊。半空间约定固定为：每行 `[h0,h1,h2,h3]` 表示 `h0·x + h1·y + h2·z + h3 ≤ 0`。

- **H-表示**（半空间/不等式形式）：用于优化约束
- **V-表示**（顶点形式）：用于渲染和凸组合参数化

走廊生成依赖 FIRI 算法进行区域膨胀，并通过 `overlap` 测试保证相邻走廊有公共体积，使连续轨迹能在其间穿行。

## 3. 代码架构 (Code Architecture)

### 3.1 模块划分和职责

```
gcopter/include/gcopter/
├── 核心优化层
│   ├── gcopter.hpp        # 主优化器 GCOPTER_PolytopeSFC
│   ├── minco.hpp          # MINCO 轨迹参数化 (S2/S3/S4 NU)
│   ├── trajectory.hpp     # 轨迹输出容器 Piece<D> / Trajectory<D>
│   └── flatness.hpp       # 微分平坦映射 FlatnessMap
├── 数学求解层
│   ├── lbfgs.hpp          # L-BFGS 无约束优化求解器
│   ├── sdlp.hpp           # Seidel 低维线性规划
│   └── root_finder.hpp    # 多项式求根（可行性/极值分析）
├── 几何处理层
│   ├── geo_utils.hpp      # 凸多面体几何工具（H↔V 转换、overlap）
│   └── quickhull.hpp      # 3D 凸包计算（用于顶点枚举）
├── 地图与走廊层
│   ├── voxel_map.hpp      # 体素占据栅格地图
│   ├── voxel_dilater.hpp  # 体素膨胀（安全裕度）
│   ├── firi.hpp           # FIRI 走廊膨胀算法
│   └── sfc_gen.hpp        # 安全飞行走廊生成（路径规划+凸覆盖+裁剪）
└── 应用层
    └── src/global_planning.cpp  # ROS 全局规划节点
```

### 3.2 数据流和调用关系

```
点云 → VoxelMap (占据栅格 + 膨胀)
     → sfc_gen::planPath (Informed RRT* 路径搜索)
     → sfc_gen::convexCover (FIRI 凸覆盖生成多面体)
     → sfc_gen::shortCut (裁剪冗余多面体)
     → GCOPTER_PolytopeSFC::setup + optimize (轨迹优化)
          ├── minco (能量与梯度传播)
          ├── flatness (推力/姿态/角速度映射)
          └── lbfgs (无约束求解)
     → Trajectory<5> (5 次多项式轨迹输出)
     → flatness::forward (轨迹 → 推力/四元数/机体角速度)
     → 控制指令 / 遥测可视化
```

### 3.3 关键类和接口

**`gcopter::GCOPTER_PolytopeSFC`**（主优化器，仅两个公开方法）
- `bool setup(...)` — 走廊归一化、H→V 转换、分段分配、变量维度初始化
- `double optimize(Trajectory<5> &traj, relCostTol)` — 运行 L-BFGS，返回最小代价

**`minco::MINCO_S3NU`**（轨迹参数化骨干）
- `setParameters(points, times)` — 构造并求解带状系统
- `getEnergy()` — 计算平滑能量
- `propogateGrad()` — 伴随法梯度反传
- `getTrajectory(Trajectory<5>&)` — 重构输出轨迹

**`flatness::FlatnessMap`**
- `forward(...)` — 平坦输出 → 推力/四元数/机体角速度
- `backward(...)` — 梯度反向传播

**`Trajectory<D>`**（输出适配器）
- `getPos/getVel/getAcc/getJer(t)` — 按全局时间评估状态
- `checkMaxVelRate/getMaxVelRate` — 可行性检查/峰值报告

---

## 4. 核心模块详解 (Core Modules)

### 4.1 gcopter.hpp — 主优化器

**文件位置**：`/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/gcopter.hpp`

头文件实现，约 860 行，单类 `GCOPTER_PolytopeSFC`。

**类型别名**：
- `PolyhedronV = Eigen::Matrix3Xd`（顶点形式）
- `PolyhedronH = Eigen::MatrixX4d`（半空间形式 `[a b c d]`，`a·x + d ≤ 0`）

**优化变量**：无约束变量 `x = [tau; xi]`，维度 `temporalDim + spatialDim`。

**costFunctional（L-BFGS 目标函数）**每次迭代组装代价与梯度：
1. `minco.setParameters(points, times)` 后调用 `getEnergy` 得平滑能量及其对系数/时间的偏导
2. `attachPenaltyFunctional` 添加软约束惩罚
3. `minco.propogateGrad` 将系数/时间梯度转为对路径点和时间的梯度
4. 加时间代价 `rho · Σ T`（梯度 `+= rho`）
5. 经 `backwardGradT/backwardGradP` 及范数限制层链式回传

**总代价** = 平滑能量 + `rho`·总时间 + 积分惩罚泛函 + 单纯形范数限制。

**attachPenaltyFunctional** 沿每段用梯形积分（`integralResolution` 个采样点）评估惩罚。每个采样点构建基向量 `beta0..beta4`（位置到 snap），用 `flatmap.forward` 得推力/四元数/角速度，对各违约量经 `smoothedL1`（C2 光滑 L1 松弛，宽度 `mu = smoothEps`）惩罚：

| 约束 | 违约量 | 权重 |
|------|--------|------|
| 位置 | 每个半空间面 `outerNormal·pos + d` | pos |
| 速度 | `‖vel‖² − v_max²` | vel |
| 机体角速度 | `‖omg‖² − omg_max²` | omg |
| 倾角 | `acos(1 − 2(qx²+qy²)) − theta_max` | theta |
| 推力 | `(thr − thrustMean)² − thrustRadi²` | thrust |

**参数约定**（内联文档）：
- `magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]`
- `penaltyWeights = [pos, vel, omg, theta, thrust]`
- `physicalParams = [mass, gravity, h_drag, v_drag, parasitic_drag, speed_smooth_factor]`

### 4.2 minco.hpp — MINCO 轨迹参数化

**文件位置**：`/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/minco.hpp`

**BandedSystem**：带状线性系统高效求解器，O(N) 复杂度。
- `factorizeLU()` — LU 分解
- `solve()` — 正向求解
- `solveAdj()` — 伴随求解（用于梯度反传）

**核心算法**：
1. **轨迹生成** (`setParameters`)：在 A 中编码边界条件、位置连续性、速度/加速度/jerk/snap 连续性；b 含边界状态与路径点位置；带状 LU 求解。
2. **能量计算** (`getEnergy`)：闭式积分（如 ∫‖jerk‖²dt），由高阶系数乘时间幂构成。
3. **梯度传播** (`propogateGrad`)：伴随法解 `Aᵀλ = ∂L/∂b`，提取对路径点的梯度，并经链式法则计算对时间的梯度。
4. **轨迹提取** (`getTrajectory`)：系数矩阵转 `Trajectory<D>`，每段含时长与降幂顺序系数。

### 4.3 trajectory.hpp — 轨迹输出容器

**文件位置**：`/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/trajectory.hpp`

**`Piece<D>`**：单个 3D 多项式段，存 `duration` 加 `CoefficientMat`（3×(D+1)，降幂存储，列 D 为常数项即起点位置）。

**`Trajectory<D>`**：`std::vector<Piece<D>>` 封装，提供 STL 式访问。实例化为 `Trajectory<3>`（cubic）、`Trajectory<5>`（quintic）、`Trajectory<7>`（septic）。

**状态评估**：`getPos/getVel/getAcc/getJer(t)` 用降幂多项式直接求值，各阶导数带正确的下降阶乘权重。

**可行性与极值分析**：
- `getMaxVelRate/getMaxAccRate`：经 `RootFinder::polySqr` 构建（归一化）导数的平方范数多项式，求导后用 `RootFinder::solvePolynomial` 找 [0,1] 驻点，取最大。
- `checkMaxVelRate/checkMaxAccRate`：更廉价的布尔可行性测试，用 `RootFinder::countRoots` 检测 `‖·‖² − limit²·t^k` 在 [0,1] 是否有根（无根即约束满足）。

### 4.4 flatness.hpp — 微分平坦映射

**文件位置**：`/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/flatness.hpp`

详见 2.3。物理参数：`mass`、`grav`、`dh`、`dv`、`cp`、`veps`。`dh/mass` 比值反复出现，体现阻力-惯性耦合；水平/垂直阻力分离允许建模各向异性气动。机体角速度由 z 轴时间导数 `dz = N_g·(j + (dh/m)·dw)` 算出，其中投影矩阵 `N_g = (I − zzᵀ)/‖zu‖`。

### 4.5 其他关键模块

**lbfgs.hpp** — L-BFGS 无约束优化求解器
- 基于 Eigen 的头文件实现，专用于光滑或分段光滑函数
- Lewis-Overton 线搜索，满足 Armijo + 弱 Wolfe 条件
- 谨慎更新策略保证非凸问题全局收敛

**geo_utils.hpp** — 凸多面体几何工具
- `enumerateVs`：H 表示 → V 表示（极对偶 + QuickHull）
- `overlap`：测试两多面体是否相交
- `findInterior`：Chebyshev 中心式可行性 LP

**firi.hpp** — FIRI 走廊膨胀
- Fast Iterative Region Inflation
- 迭代细化：椭球参数变换 → 切平面生成 → MVIE 优化
- 输出凸多面体走廊

**sfc_gen.hpp** — 安全飞行走廊生成
- `planPath`：OMPL Informed RRT* 路径搜索
- `convexCover`：FIRI 凸覆盖生成多面体
- `shortCut`：移除冗余多面体

**voxel_map.hpp** — 体素地图
- 扁平 1D 数组存储，O(1) 查询
- 占据状态：Unoccupied / Occupied / Dilated

**root_finder.hpp** — 多项式求根
- 闭式解（阶 ≤ 4）+ Sturm 序列法 + 特征值法
- 用于轨迹可行性检查和极值分析

---

## 5. 规划流程 (Planning Pipeline)

应用层入口 `src/global_planning.cpp`（ROS 节点 "global_planning_node"，主循环 1000 Hz）。

### 阶段 1：地图初始化（mapCallBack）
- **输入**：点云 `sensor_msgs::PointCloud2`
- 一次性处理，解析点云、过滤 NaN/Inf、填充占据栅格、按 `dilateRadius` 膨胀障碍
- **输出**：`VoxelMap` 占据栅格

### 阶段 2：目标采集（targetCallBack）
- **输入**：目标位姿 `geometry_msgs::PoseStamped`
- 采集起止点对（需 2 点），Z 坐标由位姿朝向计算，接受前做碰撞检查
- **输出**：起点、终点

### 阶段 3：路径规划（plan）
- **输入**：起点、终点、地图边界
- `sfc_gen::planPath()` 在体素地图中搜索无碰撞路径（0.01 分辨率）
- **输出**：航点路线

### 阶段 4：走廊生成
- **输入**：路线 + 体素地图表面点
- `convexCover()`（参数 7.0 / 3.0，尺寸/裕度）拟合凸多面体 → `shortCut()` 移除冗余
- **输出**：H 表示多面体序列

### 阶段 5：轨迹优化（GCOPTER）
- **输入**：边界条件（端点零速度/加速度）、幅值界、惩罚权重 chiVec、物理参数、时间权重、平滑 epsilon、积分分辨率
- `setup()` + `optimize()` 最小化加权代价至 `relCostTol` 收敛
- **输出**：5 次多项式轨迹 `Trajectory<5>`

### 阶段 6：实时跟踪（process，1000 Hz）
- 按经过时间计算当前轨迹状态
- 正向平坦变换：(速度, 加速度, jerk) → (推力, 四元数, 机体角速度)
- **输出**：遥测发布（速度幅值、推力、倾角、机体角速度幅值）+ 位置可视化

---

## 6. 配置参数 (Configuration)

配置文件 `global_planning.yaml`，22 个参数经 ROS 私有命名空间加载。

### 6.1 ROS 接口
- `MapTopic`: `/voxel_map` — 体素地图订阅
- `TargetTopic`: `/move_base_simple/goal` — 目标位姿

### 6.2 地图与环境
- `DilateRadius`: 0.5 m — 障碍安全缓冲（增大更保守）
- `VoxelWidth`: 0.25 m — 地图分辨率（越小越精确但越慢）
- `MapBound`: [-25, 25, -25, 25, 0, 5] m — 规划空间界

### 6.3 规划算法
- `TimeoutRRT`: 0.02 s — RRT 采样超时（找不到路径时增大）

### 6.4 飞行器运动约束
- `MaxVelMag`: 4.0 m/s — 最大速度幅值
- `MaxBdrMag`: 2.1 — 最大机体角速度幅值
- `MaxTiltAngle`: 1.05 rad (~60°) — 最大倾角

### 6.5 飞行器物理参数
- `VehicleMass`: 0.61 kg
- `GravAcc`: 9.8 m/s²
- `MinThrust` / `MaxThrust`: 2.0 / 12.0 N（推重比约 20:1）

### 6.6 气动阻力系数
- `HorizDrag`: 0.70 — 水平阻力
- `VertDrag`: 0.80 — 垂直阻力
- `ParasDrag`: 0.01 — 寄生阻力
- `SpeedEps`: 0.0001 — 速度 epsilon（数值稳定）

### 6.7 优化代价函数
- `WeightT`: 20.0 — 时间权重（越高越快但越激进）
- `ChiVec`: [1e4, 1e4, 1e4, 1e4, 1e5] — 惩罚权重 [位置, 速度, 角速度, 倾角, 推力]

### 6.8 数值优化
- `SmoothingEps`: 1e-2 — 非光滑惩罚的平滑 epsilon
- `IntegralIntervs`: 16 — 沿轨迹数值积分区间数
- `RelCostTol`: 1e-5 — 相对代价收敛容差

### 6.9 调参建议

| 目标 | 调整方式 |
|------|---------|
| 激进飞行 | 增大 MaxVelMag/MaxBdrMag/MaxTiltAngle，减小 WeightT |
| 保守飞行 | 增大 DilateRadius/WeightT，减小速度/加速度限制 |
| 加快规划 | 增大 TimeoutRRT/RelCostTol，减小 IntegralIntervs |
| 更平滑轨迹 | 增大 WeightT，减小 ChiVec 惩罚 |
| 不同飞行器 | 据系统辨识调整质量、推力界、阻力系数 |

---

## 7. 扩展开发指南 (Development Guide)

### 7.1 如何添加新的约束

约束在 `gcopter.hpp` 的 `attachPenaltyFunctional` 中以软惩罚形式添加。步骤：

1. **在积分采样循环内**，从 `flatmap.forward` 输出（或 `beta0..beta4` 状态基）计算新的违约量，形式应为 `violation = quantity − limit`。
2. **用 `smoothedL1(violation, ...)`** 做 C2 光滑松弛，乘以对应惩罚权重累加到代价。
3. **计算违约量对状态（pos/vel/acc/jer 及 psi）的梯度**，经 `flatmap.backward` 回传到 `gradC`/`gradT`。
4. **扩展参数向量**：在 `magnitudeBounds` 加入新限值，在 `penaltyWeights`（ChiVec）加入新权重，并在 `setup` 中相应解析。

参考现有的速度/角速度/倾角/推力约束实现作为模板，保持 `smoothedL1` + `flatmap.backward` 的梯度链一致性。

### 7.2 如何修改优化目标

- **改变平滑阶数**：将 `minco::MINCO_S3NU`（最小 jerk）换成 `MINCO_S2NU`（最小加速度）或 `MINCO_S4NU`（最小 snap），并相应调整 `Trajectory<D>` 的阶数 D（3/5/7）和边界条件维度。
- **调整时间-平滑权衡**：修改时间权重 `rho`（配置中 WeightT）。代价 = 平滑能量 + `rho`·总时间 + 惩罚。
- **启用点质量模型**：按 README 提示修改惩罚泛函代码，跳过完整平坦映射以加快计算（牺牲阻力建模精度）。
- **修改 L-BFGS 行为**：调 `lbfgs_parameter_t`（`mem_size`、`past`、`delta`）。注意对含惩罚项的非光滑代价，保持 `g_epsilon = 0.0` 并依赖 delta 测试。

### 7.3 常见问题与调试技巧

**优化失败 / 返回 INFINITY**：
- 检查 `setup` 是否返回 false（顶点枚举失败，走廊多面体退化）。
- 用 `geo_utils::overlap` 验证相邻走廊确实重叠；不重叠会导致 `forwardP` 无法构造可行初值。
- 检查 L-BFGS 返回码，用 `lbfgs_strerror` 解码。`LBFGSERR_MAXIMUMLINESEARCH` 常意味着梯度有误或缩放糟糕。

**轨迹违反动力学约束**：
- 增大对应的 ChiVec 惩罚权重，或减小 `SmoothingEps`（更接近硬约束但更难优化）。
- 用 `Trajectory::getMaxVelRate/getMaxAccRate` 报告实际峰值，`checkMaxVelRate/checkMaxAccRate` 做快速可行性检查。

**梯度不正确（最常见的扩展 bug）**：
- 新增约束后，务必确保 `flatmap.backward` 的梯度链完整。可用有限差分对比 `costFunctional` 的解析梯度做单元验证。
- 注意 MINCO 系数约定：内部按行升幂存储，`Trajectory` 用降幂；`getTrajectory` 中的 `transpose().rowwise().reverse()` 是衔接关键。

**数值奇异**：
- 零速时阻力计算靠 `veps`（SpeedEps）防奇异；倾角接近 180° 时 `tilt_den`/`omg_den` 奇异，倾角约束应防止进入该区域。
- `findInterior` 对零范数约束行会产生 NaN/Inf，确保走廊多面体不含退化行。

**实时性能**：
- 减小 `IntegralIntervs`（积分采样数）和 FIRI 迭代次数（默认 4）可加速，但降低约束满足精度与走廊质量。
- `locatePieceIdx` 按引用修改 `t`，在轨迹采样热路径中避免误用；优先用 O(1) 的 `getJuncPos/Vel/Acc` 做交界点查询。

---

## 8. 模块文件索引

所有核心模块的绝对路径：

**核心优化层**
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/gcopter.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/minco.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/trajectory.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/flatness.hpp`

**数学求解层**
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/lbfgs.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/sdlp.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/root_finder.hpp`

**几何处理层**
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/geo_utils.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/quickhull.hpp`

**地图与走廊层**
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/voxel_map.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/voxel_dilater.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/firi.hpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/include/gcopter/sfc_gen.hpp`

**应用层**
- `/home/code/Plan/GCOPTER-HTQR/gcopter/src/global_planning.cpp`
- `/home/code/Plan/GCOPTER-HTQR/gcopter/config/global_planning.yaml`

---

## 附录：与博士论文的对应关系

本代码库实现了王哲沛博士论文《A Geometrical Approach to Multicopter Motion Planning》中的核心算法。建议配合论文阅读：

- **第2章 轨迹表示** → `minco.hpp` 实现
- **第3章 几何约束优化** → `gcopter.hpp` 的约束消除技术
- **第4章 微分平坦性** → `flatness.hpp` 实现
- **第5章 走廊生成** → `firi.hpp` 和 `sfc_gen.hpp` 实现
- **第6章 完整系统** → `global_planning.cpp` 集成

论文位置：`/home/code/Plan/GCOPTER-HTQR/Thesis - ZhepeiWang - Chinese - A Geometrical Approach to Multicopter Motion Planning.pdf`

---

**文档生成信息**：
- 生成时间：2026-06-01
- 分析模块数：16
- 并行 Agents：17
- 总消耗 tokens：约 444k（子agents）

本文档由 GCOPTER 代码库系统分析自动生成，涵盖核心概念、架构设计、模块详解、规划流程、配置参数和扩展开发指南。

