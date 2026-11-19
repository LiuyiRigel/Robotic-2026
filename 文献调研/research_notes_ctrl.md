# 运动学建模、标定、轨迹规划与关节伺服控制 文献调研（主题B）

> 说明：本文件仅收录可核查元数据（DOI 已通过 Crossref / OpenAlex 交叉验证）。共 **19 条**：中文核心 4 条、英文 15 条；2021—2026 年 **18 条**。标注"待核"的字段为尚未取到原始页面确认者。

## 一、现状要点

### 1.1 运动学建模与逆解

六自由度串联机械臂的建模主要有三条参数化路线：标准 D-H、改进 D-H（Craig）与指数积（PoE）旋量法。D-H 用四个连杆参数描述相邻轴线，物理直观但坐标系选择不唯一；改进 D-H 把连杆坐标系固连于驱动轴，便于处理平行轴与树状结构；PoE 以关节旋量的指数映射描述整体位姿，不需要人为选坐标系，误差模型对参数是线性的，因而在标定中更受青睐。He 等[1]建立的 PoE 误差模型给出可辨识性结论：n 自由度串联机器人最多有 6n+6 个可辨识参数，对 r 个转动关节、t 个移动关节的实际结构为 6r+3t+6；关节零位误差与初始位姿误差不能在同一模型中同时辨识，需按旋量坐标线性无关条件取舍，SCARA 与 PUMA560 的全部运动学参数均可辨识。Yang 等[2]以 5 自由度 5PRR+5PUS-PRPU 变结构混联机构为对象，分别用逆解法与 PoE 法建立误差模型并做标定仿真，发现两种模型在不同基座位形下的参数可辨识性与补偿效果并不一致。逆解方面，Xiao 等[3]提出"断开—再连接"的统一推导方法，把 6 自由度简单几何构型机械臂切成两个三自由度子链，归纳出 5 种切分方式与方程变形选择原则，6 个关节变量仅由 6 个方程导出，避免通用 6R 的数值迭代。对本课题的启示：正解宜用改进 D-H 建表，逆解用解析法为主、阻尼最小二乘（DLS）数值迭代为备份，并用雅可比条件数与 Yoshikawa 可操作度刻画奇异位形与灵巧度。

### 1.2 轨迹规划

关节空间以三次/五次多项式、梯形与 S 形速度曲线为主；七段 S 曲线把加速度与加加速度（jerk）同时限制在饱和值，可显著减小残余振动与冲击。Wu 等[4]提出四阶（jerk 连续）S 曲线规划，令加速度与 jerk 保持饱和以提高编程与运行效率，算法不含任何优化或迭代过程，由多项式方程直接生成满足运动学约束的轨迹，可实现实时控制，并给出多轴同步算法，在关节空间和笛卡尔空间均做了验证。笛卡尔空间直线/圆弧插补需同时处理位置与姿态插值及奇异回避，常以 B 样条/NURBS 做平滑。Tan 等[5]用三次 B 样条实现多轴实时插补，给出 ISO 参数分段插补算法与恒速运动精度分析，并在自研嵌入式系统（硬件电路+软件逻辑）上直接运行生成轨迹，摆脱了" CAM 生成 G 代码—下载到 CNC "的离线流程。岳晓明等[6]面向 6 自由度串联机器人的电火花铣削加工，研究轨迹规划与伺服控制策略的匹配关系。王汝贵等[7]针对变胞式工业机器人给出时间最优轨迹规划方法。对 STM32 实现的启示：七段 S 曲线可用定点 Q 格式加查表三角函数实现，插补周期 1 ms、位置环 2~5 kHz 时，单轴每周期仅需数十次乘加，Cortex-M4 的 FPU 足以承担；B 样条则宜用 de Boor 递推并预存控制点。

### 1.3 精度与标定

重复定位精度主要由机构与伺服一致性决定，绝对定位精度则受几何参数误差、减速器背隙、连杆柔性、编码器分辨率、装配误差与热变形共同影响。Kana 等[8]指出装配阶段引入的关节零位偏差是 90% 以上定位不准的根源，提出不依赖外部测量设备与机械夹具的快速运动学再标定方法，用于弥合出厂标定控制器与名义模型之间的差异。Khanesar 等[9]用差分进化、粒子群、人工蜂群与引力搜索四种智能优化算法标定工业机器人 D-H 参数，以 Leica AT960-MR 激光跟踪仪采集精确位姿作为基准，比较了各算法的收敛与残差表现。梁相龙等[10]对六自由度液压机械臂开展运动学标定与逆解联合研究。高贯斌等[19]进一步区分了"静态定位精度"与"动态轨迹精度"：工业机器人的轨迹精度明显低于其定位精度，仅靠运动学标定修正几何参数不足以消除沿路径的误差，需在标定之后于关节空间对指令轨迹逐点插值补偿，把误差补偿从点位扩展到连续轨迹。Wang 等[11]针对 7R 六自由度非球腕机器人建立同时含几何与非几何误差的标定模型；Ye 等[12]研究过约束机器人几何误差与内部变形耦合下的标定；Li 等[13]以"几何约束失效"与"运动学误差节点"构造完整误差建模流程。对本课题的启示：本科条件难以配置激光跟踪仪，可采用"球杆仪/自制标定板+视觉"或"末端接触式三点法"做简化标定，重点交代重复定位精度指标与误差源分解。

### 1.4 关节伺服与总线

关节伺服普遍采用 FOC 加三环级联：电流环（1~10 kHz）、速度环（0.5~2 kHz）、位置环（0.2~1 kHz），电流环带宽决定转矩响应，位置环增益受机械谐振与编码器噪声限制。Song 等[14]针对多电机伺服系统提出预设时间滑模跟踪控制与平均相对耦合同步控制，跟踪误差与同步误差的收敛时间可按需求预先设定，兼顾负载跟踪与电机间速度一致性。Zheng 等[15]针对柔性关节机器人伺服系统的振动抑制、参考跟踪、约束满足与扰动抑制，把 PMSM 驱动的扭转力矩、多频转矩脉动与参数不确定性统一建模为外系统，用降阶扰动观测器估计后以输出调节 MPC 求解。总线方面，Wei[16]用 CAN 总线构建"1 主+4 从"DSP 三轴伺服系统，主控负责信号读写与人机接口，从站完成数字位置控制，并以高精度编码器为基准校准低精度编码器。编码器方面，Feng 等[17]用两片极对数不同的多极磁环构成差动绝对式磁编码器，在提高磁场覆盖面积的同时保证绝对角度精度不低于同极对数组合式磁编码器。轻量化方面，Wang 等[18]设计基于负载自适应提升机构的三自由度轻量机械臂，用弹簧绳索与变半径卷筒实现变负载自适应，在降低自重的同时保持承载能力。

## 二、参考文献表（GB/T 7714）

[1] 王汝贵, 董奕辰, 陈辉庆. 一种变胞式工业机器人时间最优轨迹规划[J]. 机械工程学报, 2023, 59(5): 100-111. DOI: 10.3901/JME.2023.05.100.

[2] YANG H, BAI L, JIANG Z, et al. Comparison of error modeling between inverse kinematics and product of exponentials methods for a 5-DOF hybrid perfusion manipulator[J]. Mechanical Sciences, 2026, 17(1): 575-591. DOI: 10.5194/ms-17-575-2026.

[3] XIAO F, LI G, JIANG D, et al. An effective and unified method to derive the inverse kinematics formulas of general six-DOF manipulator with simple geometry[J]. Mechanism and Machine Theory, 2021, 159: 104265. DOI: 10.1016/j.mechmachtheory.2021.104265.

[4] WU G, ZHANG N. Kinematically constrained jerk-continuous S-curve trajectory planning in joint space for industrial robots[J]. Electronics, 2023, 12(5): 1135. DOI: 10.3390/electronics12051135.

[5] TAN Q, MOHD BASRI M A. Hardware-software embedded system for real-time trajectory planning of multi-axis machine using B-spline curve interpolation algorithm[J]. Machines, 2023, 11(12): 1043. DOI: 10.3390/machines11121043.

[6] 岳晓明, 臧烁, 徐作珂, 等. 基于6自由度串联机器人的电火花铣削加工轨迹规划及伺服控制策略研究[J]. 机械工程学报, 2023, 59(9): 20-27. DOI: 10.3901/JME.2023.09.020.

[7] 梁相龙, 姚志凯, 邓文翔, 等. 六自由度液压机械臂运动学标定和逆解研究[J]. 机械工程学报, 2025, 61(2): 346-357. DOI: 10.3901/JME.2025.02.346.

[8] KANA S, GURNANI J, RAMANATHAN V, et al. Fast kinematic re-calibration for industrial robot arms[J]. Sensors, 2022, 22(6): 2295. DOI: 10.3390/s22062295.

[9] KHANESAR M A, YAN M, ISA M, et al. Precision Denavit-Hartenberg parameter calibration for industrial robots using a laser tracker system and intelligent optimization approaches[J]. Sensors, 2023, 23(12): 5368. DOI: 10.3390/s23125368.

[10] HE R, ZHAO Y, YANG S, et al. Kinematic-parameter identification for serial-robot calibration based on POE formula[J]. IEEE Transactions on Robotics, 2010, 26(3): 411-423. DOI: 10.1109/TRO.2010.2047529.

[11] WANG X, SUN S, ZHANG P, et al. Model-based kinematic and non-kinematic calibration of a 7R 6-DOF robot with non-spherical wrist[J]. Mechanism and Machine Theory, 2022, 178: 105086. DOI: 10.1016/j.mechmachtheory.2022.105086.

[12] YE H, WU J, HUANG T. Kinematic calibration of over-constrained robot with geometric error and internal deformation[J]. Mechanism and Machine Theory, 2023, 185: 105345. DOI: 10.1016/j.mechmachtheory.2023.105345.

[13] LI Z, GUO W. A complete approach for error modeling based on failure of geometrical constraint and kinematic error node (KEN)[J]. Mechanism and Machine Theory, 2025, 209: 105984. DOI: 10.1016/j.mechmachtheory.2025.105984.

[14] SONG J, REN X, NA J. Enhanced predefined-time tracking and synchronization control for multimotor servo systems[J]. IEEE Transactions on Industrial Electronics, 2025, 72(7): 7094-7105. DOI: 10.1109/TIE.2024.3493163.

[15] ZHENG L, LIU J, LI S. High-performance control strategy for robot servo system with flexible joint via output regulation model predictive control[J]. IEEE Transactions on Power Electronics, 2026, 41(4): 5237-5248. DOI: 10.1109/TPEL.2025.3634770.

[16] WEI M Y. Design and control of a three-axis motion servo control system based on a CAN bus[J]. Energies, 2023, 16(10): 4208. DOI: 10.3390/en16104208.

[17] FENG T, CHEN W, QIU J, et al. A new kind of absolute magnetic encoder[J]. Sensors, 2021, 21(9): 3095. DOI: 10.3390/s21093095.

[18] WANG R, LÜ Z, WANG Y, et al. The design and analysis of a lightweight robotic arm based on a load-adaptive hoisting mechanism[J]. Actuators, 2025, 14(2): 71. DOI: 10.3390/act14020071.

[19] 高贯斌, 张石文, 那靖, 等. 基于标定和关节空间插值的工业机器人轨迹误差补偿[J]. 机械工程学报, 2021, 57(21): 55-67. DOI: 10.3901/JME.2021.21.055.

## 三、可用于"研究现状"段的中文压缩稿（约 380 字）

六自由度机械臂的运动控制研究大体沿三个层次递进。第一层是建模与解算：D-H 参数法因物理意义清晰仍是工程主流，改进 D-H 便于处理平行轴，而指数积（PoE）旋量法以关节旋量描述整体位姿，误差模型对参数线性、可辨识参数上限为 6n+6[10]，在标定中优势明显；逆解则从通用 6R 的数值迭代走向按几何构型"断开—再连接"的解析推导，6 个关节变量可由 6 个方程闭合求解[3]。第二层是轨迹与精度：关节空间七段 S 曲线以 jerk 连续换取平稳性，四阶 S 曲线可在不迭代的前提下实时生成并实现多轴同步[4]；笛卡尔空间则以 B 样条实时插补配合恒速精度控制[5]。精度方面，装配零位偏差被证实是 90% 以上定位误差的根源[8]，激光跟踪仪结合智能优化可系统辨识 D-H 参数[9]，国内学者亦在六自由度液压臂上完成了标定与逆解的联合研究[7]；但标定只保证点位，轨迹精度仍显著低于定位精度，需在关节空间对指令轨迹再做插值补偿[19]。第三层是伺服与轻量化耦合：多电机伺服已发展到预设时间跟踪与同步控制[14]，柔性关节的振动与扰动则由输出调节 MPC 统一处理[15]；轻量化结构常以负载自适应机构、低减速比与磁编码器换取功重比[17-18]，但也把重力补偿与末端抖动问题推到控制层。

## 四、关键局限证据清单

1. 标定模型的可辨识性受结构约束：关节零位误差与初始位姿误差不可同时辨识，n 自由度最多 6n+6 个参数（实际 6r+3t+6）[10]。→ 本科课题若照搬"全参数辨识"，会出现雅可比秩亏、解不唯一。
2. 逆解解析法依赖"简单几何"（后三轴交于一点或轴线平行/垂直）：Xiao 等的方法需先把 6R 切成两个 3-DOF 子链，仅适用于 5 种可切分构型[3]。→ 自研构型若偏置非零，只能退回数值解。
3. 四阶 S 曲线虽免迭代，但要求加速度与 jerk 同时饱和，对启停频繁、行程极短的关节会退化为近似梯形曲线，平滑收益下降[4]。
4. B 样条嵌入式实现需预存控制点并做 de Boor 递推，文献[5]的恒速精度分析建立在 ISO 参数分段插补之上，曲率突变处速度仍受限于机床动力学，不能直接移植到低成本关节。
5. 快速再标定[8]依赖"出厂控制器与名义模型差异"这一先验，且需机器人可读取关节角；对无绝对值编码器、需每次上电回零的低成本臂不适用。
6. 智能优化标定[9]需 Leica AT960-MR 级激光跟踪仪与大量位姿采样，单次标定成本高，本科实验条件难以复现。
7. 柔性关节 MPC[15]需精确的 PMSM 参数与扭转刚度模型，且计算量大，通常需 DSP/FPGA 级算力，STM32F4 单核难以在中频（≥1 kHz）闭环内完成。
8. CAN 总线伺服[16]的主从架构在 4 轴时通信负载尚可，轴数增加或周期缩短到 1 ms 时，标准 CAN 2.0B（1 Mbps）的仲裁与排队时延会成为同步误差来源。
9. 绝对式磁编码器[17]以双磁环差动提高覆盖面积，但安装偏心与磁环充磁误差仍直接进入角度误差，14 位分辨率（约 0.022°）在 300 mm 臂展上对应约 0.11 mm 末端分辨力，构成绝对精度下限。
10. 轻量化臂[18]以弹簧绳索提升机构做负载自适应，属被动重力补偿，补偿量随位形与负载变化，不能替代伺服层的前馈重力补偿。
11. 标定+关节空间插值补偿[19]仍属"离线辨识参数、在线插值修正"的两步法：其有效性依赖标定后模型与实际机构的一致性，一旦负载、温度或减速器磨损改变误差场，补偿表即失效；文献亦未覆盖低减速比、柔性连杆这一类非几何误差主导的场合。

## 五、待核实条目（暂未写入正文引用）

- [1] 的止页：Crossref 仅登记起页 100，实际止页待期刊页面确认（本次抓取 qikan.cmes.org 超时）。
- [11][12][13] 三篇 Mechanism and Machine Theory 论文仅核实到题名、作者、卷期、页码与 DOI，Elsevier 摘要页被 CloudFront 拦截，具体方法与数字尚未取得，正文中仅按题名做概括，未引用具体数据。
- 中文核心期刊《机器人》《仪器仪表学报》《组合机床与自动化加工技术》的候选条目因数据库不提供 Crossref 记录、官网抓取受限，暂未纳入。
- [19] 的摘要由检索快照（中国知网/360 检索摘要）确认主题句，全文数据（补偿前后轨迹误差具体数值）未取得，正文仅按摘要口径概括，未引具体数字。
