# `real_lab` Sim2Real 场景与任务联合设计文档

- **目标世界**：`worlds/sim2real/real_lab.world`（1:1 对齐真实实验室点云，全图 `x ∈ [-11.66, 37.75]`、`y ∈ [-9.05, 21.00]`）
- **任务配置**：`model/Data_exchange/vlm_prompt/lab/real_lab.json`
- **状态**：第二轮设计稿（已核对本地 Gazebo 资产库、会议桌布局、`table lamp + plant` 上下文及 WeDetect 映射表）

---

## 1. 本地 Gazebo 室内资产盘点（`~/.gazebo/models/`，共 648 个模型）

实验室电脑本地已包含约 280 个室内相关模型，**基础办公/实验室/仓库素材非常齐全**，暂无需大量外部下载（若后续需要白板、打印机、饮水机、红色灭火器等细分道具，可再按需补充 Gazebo Fuel 模型）。

### 核心候选模型实测尺寸（经 Collada 节点变换换算为米，X × Y × Z）
| 类别 | 推荐模型 | 实测尺寸（m） | 备注 |
| --- | --- | --- | --- |
| **会议桌/办公桌** | `table_conference_1` | `3.08 × 1.10 × 0.67` | 桌面仅 `0.67 m` 高，对低视角相机很友好 |
| | `table_conference_2` | `2.71 × 1.21 × 0.70` | `new.world`（论文办公室）同款会议桌，高 `0.70 m` |
| | `table_conference_3` | `1.22 × 3.66 × 0.74` | 更长的会议桌 |
| | `desk_brown` | `1.77 × 0.80 × 0.72` | 常规单人/双人办公桌 |
| **椅/沙发** | `office_chair` | `0.60 × 0.63 × 0.99` | 办公椅（`new.world` 同款） |
| | `sofa_set_1 / 2 / 3` | `2.00~2.42 × 0.73~0.99 × 0.68~0.86` | 备用休息区沙发 |
| **柜/架** | `bookshelf_large` | `1.66 × 0.43 × 1.84` | 贴墙高书架/货架 |
| | `bookshelf` | `0.90 × 0.40 × 1.20` | 中型书架 |
| | `file_cabinet_small / large` | `0.62 × 0.38~0.75 × 0.65` | 文件柜 |
| **桌面电子/文具** | `monitor_1 / 2 / 3 / 4` | `0.60 × 0.10 × 0.44` | 桌面显示器（立式易检出） |
| | `computer_case`、`keyboard`、`mouse` | `keyboard` 仅高 `0.02 m` | **仅作场景视觉丰富度摆件，不列入 JSON 识别项** |
| | `file_1`、`file_2`、`file_box_1~4`、`book_*` | 立式文件夹/文件盒 | 桌面丰富度与 `file folder` 线索 |
| **台灯（`ctx_left`）** | `lamp_table_metal` | `0.30 × 0.30 × 0.40` | **首选桌面台灯**：高度 `0.40 m`，远比 `small` 醒目 |
| | `lamp_table_small` | `0.17 × 0.17 × 0.22` | 备选小台灯 |
| | `lamp_table_large` | `0.58 × 0.58 × 0.75` | 偏大 |
| **绿植（`ctx_right`）** | `plant_2` | `0.29 × 0.34 × 0.56` | **首选桌面绿植**：占地紧凑、高 `0.56 m`，适合放桌上杯旁 |
| | `plant_1` | `0.78 × 0.80 × 0.63` | 落地大盆栽（`env06` 已验证 WeDetect 稳定检出），可放桌旁地面 |
| | `plant_3` | `0.55 × 0.53 × 0.29` | 矮宽盆栽（⚠️ `pot_flower` 网格异常，禁用） |
| **目标** | `cup_yellow` | `0.123 × 0.082 × 0.077` | 目标黄色杯子（`build_design.py` 实测尺寸） |
| **仓库与走廊负线索** | `ladder_75deg_0rails` | `0.95 × 1.55 × 2.52` | 仓库梯子（`env05` 已验证） |
| | `cardboard_box` | `0.50 × 0.40 × 0.30` | 纸箱（可堆叠） |
| | `fire_hydrant` | `0.42 × 0.42 × 0.93` | 走廊消防栓 |
| | `construction_cone` | `0.50 × 0.50 × 1.09` | 走廊锥桶（也有 `construction_cone_small`） |

---

## 2. WeDetect 标签映射检查结果（关键！）

在 `src/lste_core/scripts/utils/wedetect_backend.py` 中，启动脚本未覆盖 `~wedetect_label_map`，全链路使用 `WeDetectBackend.DEFAULT_LABEL_MAP`（当前仅 14 项）。未命中的英文词会直接以英文送入以中文类别为主训练的 WeDetect 文本塔，影响召回率。

> 注意：`lste_score_node` 的打分权重为 `w_target=0.2, w_env=0.3, w_ctx=0.5`，**`S_ctx` 占总分 50%**，因此 `table lamp` 与 `plant` 的映射和可检出性直接决定能否稳定 `LOCKED`（`TARGET_LOCK_TOTAL_MIN=0.58`）。

| `real_lab.json` 词汇 | 角色 | 当前 `DEFAULT_LABEL_MAP` 状态 | 建议补充映射 |
| --- | --- | --- | --- |
| `yellow cup` | `target` | ✅ 已映射 `"黄色杯子"` | 保持 |
| `desk` | `env` / `key_objects` | ✅ 已映射 `"书桌"` | 建议同步补 `"conference table": "会议桌"`, `"office desk": "办公桌"`, `"table": "桌子"` |
| `monitor` | `env` / `key_objects` | ✅ 已映射 `"电脑显示器"` | 保持 |
| `chair` | `env` / `key_objects` | ✅ 已映射 `"椅子"` | 保持 |
| `file folder` | `key_objects` | ✅ 已映射 `"文件夹"` | 保持 |
| `fire hydrant` | `negative_clues` | ✅ 已映射 `"消防栓"` | 保持 |
| `bookshelf` | `env` / `key_objects` | ❌ **缺失** | `"bookshelf": "书架"`, `"shelf": "货架"` |
| `construction cone` | `negative_clues` | ❌ **缺失** | `"construction cone": "交通锥"`, `"traffic cone": "交通锥"` |
| `ladder` | `negative_clues` | ❌ **缺失** | `"ladder": "梯子"` |
| `cardboard box` | `negative_clues` | ❌ **缺失** | `"cardboard box": "纸箱"` |
| `table lamp` | `target_ctx.left` | ❌ **缺失** | `"table lamp": "台灯"`, `"desk lamp": "台灯"`, `"lamp": "台灯"` |
| `plant` | `target_ctx.right` | ❌ **缺失** | `"plant": "盆栽"`, `"potted plant": "盆栽"` |

---

## 3. 核心房间 `E3` 的“中岛会议桌”布局设计（参考论文图 3）

- **房间尺寸**：`E3` 内部 `x ∈ [27.65, 37.59]`（东西长 **`9.94 m`**），`y ∈ [9.68, 15.73]`（南北宽 **`6.05 m`**），西侧门洞 `y ∈ [11.16, 12.96]`（宽 `1.80 m`）。
- **中岛会议桌排布**：
  - 选用 `table_conference_2`（`2.71 × 1.21 × 0.70 m`，`new.world` 同款）在 `E3` 中轴线（`y ≈ 12.70`）横向排成一排 **2 张**（总长 `5.42 m`，覆盖 `x ∈ [30.0, 35.4]`，进深 `y ∈ [12.10, 13.31]`）：
    - **西侧进门缓冲**：`x ∈ [27.65, 30.0]` → 净宽 **`2.35 m`**（宽敞进门区）；
    - **东侧绕行走廊**：`x ∈ [35.4, 37.59]` → 净宽 **`2.19 m`**；
    - **南侧通行走廊**：`y ∈ [9.68, 12.10]` → 净宽 **`2.42 m`**（即使桌边放一排 `office_chair` 占 `0.65 m`，仍留有 **`1.77 m`** 车行通道）；
    - **北侧通行走廊**：`y ∈ [13.31, 15.73]` → 净宽 **`2.42 m`**（放椅子后仍留 **`1.77 m`** 车行通道，墙边还可紧贴放 `bookshelf_large` 进深 `0.43 m`，剩余通道仍 **`1.34 m`**，完全满足 Pro3 `0.70 m` 通行需求）。
  - **桌面布置**：
    - 常规工位（靠近门口的第 1 张会议桌及第 2 张桌背面）：成排 `monitor_1/2/3`、`computer_case`、`keyboard`、`mouse`、`file_1/2`、`file_box_1`、`office_chair`，保证视觉丰满且稳定触发 `S_env`；
    - 目标工位（第 2 张会议桌面向通道的一侧前沿，`z = 0.70 m`）：中央放 **`cup_yellow`**，左侧放 **`lamp_table_metal`**（高 `0.40 m`，台灯），右侧放 **`plant_2`**（高 `0.56 m`，桌面绿植；如需更强远距检出，可在桌旁地面再配 `plant_1` 或直接改用桌面 `plant_2` 实测验证）。

---

## 4. 全图房间与走廊规划总表

| 区域 | 内部坐标范围 | 门洞 | 角色 | 主要物体配置 | 预期状态机行为 |
| --- | --- | --- | --- | --- | --- |
| **走廊** | 环廊 `1.77~1.80 m`<br>中轴 `3.09 m` | — | 负分快速通道 | 贴墙放置 `construction_cone`、`fire_hydrant`（保持通道 ≥ `1.4 m`） | 负线索压低 `S_env`，保持 `PASS` |
| **`E3`** | `x∈[27.65, 37.59]`<br>`y∈[9.68, 15.73]` | 西口 `1.80 m` | **核心目标房** | 中央一排 2 张 `table_conference_2` + 四周通行走廊 + `office_chair` / `monitor_1~3` / `keyboard` / `mouse` / `file_box` / 贴墙 `bookshelf_large`；目标位 **`cup_yellow` + `lamp_table_metal` + `plant_2`** | `Sus-C` → `Sus-B` → `Sus-A` → `LOCKED` |
| **`E12`** | `x∈[27.65, 37.59]`<br>`y∈[-0.95, 9.54]` | 西口 `1.735 m` | **负分仓库** | 门口视野内放 `ladder_75deg_0rails` + `cardboard_box` 堆（+ `BoxesLargeOnPallet` 靠内） | 门口观望打低分跳过 |
| **`S1`** | `x∈[0.33, 3.92]`<br>`y∈[-5.26, -1.10]` | 北门 `0.90 m` | **负分小仓库** | `cardboard_box` 堆 + `ladder_75deg_0rails`（留 `1.2 m` 进门缓冲） | 门口观望打低分跳过 |
| **`W`** | `x∈[-11.51, -1.58]`<br>`y∈[-0.95, 10.74]` | 东口 `1.515 m` | **负分大仓库** | `ladder_75deg_0rails` + `cardboard_box` 堆 + `BoxesLargeOnPallet` | 门口观望打低分跳过 |
| **`S23`** | `x∈[10.02, 18.48]`<br>`y∈[-8.91, -1.10]` | 北门 **`0.85 m`** | **诱导房** | `table_conference_2` 或贴墙 `desk_brown` + `office_chair` + `monitor_1~3` + `keyboard/mouse` + `bookshelf_large`（**无台灯、无绿植、无黄杯**） | 触发 `S_env` 进房查看后离开 |
| **`N23`** | `x∈[13.52, 24.07]`<br>`y∈[13.10, 20.85]` | 南门 `0.90 m` | **诱导房** | 同上（**无台灯、无绿植、无黄杯**） | 触发 `S_env` 进房查看后离开 |
| **`S4`** | `x∈[20.97, 25.57]`<br>`y∈[-8.91, -1.10]` | 北门 `1.40 m` | **空房** | 无 | 空房对照 |
| **`N1`** | `x∈[0.67, 7.38]`<br>`y∈[13.10, 20.85]` | 南门 `0.90 m` | **空房** | 无 | 空房对照 |

---

## 5. 待确认后即可动工的点

1. **是否把第 2 节缺失的中文映射直接补进 `src/lste_core/scripts/utils/wedetect_backend.py` 的 `DEFAULT_LABEL_MAP`？**（特别是 `table lamp -> 台灯`、`plant -> 盆栽`、`ladder -> 梯子`、`cardboard box -> 纸箱`、`construction cone -> 交通锥`、`bookshelf -> 书架`）
2. **诱导房 `S23` / `N23` 的桌子摆放风格**：也采用房间中央排 `table_conference_2` 留周围走廊，还是贴墙放 `desk_brown`？
3. **是否需要额外下载任何 Gazebo Fuel 室内模型**（如白板、打印机、饮水机、灭火器）？如不需要，现有 648 个本地模型已完全够用，确认后我可直接生成脚本写入 `real_lab.world` 并渲染俯视/门口视角图做核验。
