# 原反馈律的 gamma 参数设计重构稿

这是一份独立审阅稿，原 `texdocs` 论文未被替换。它围绕“扰动预算与误差要求 -> 性能参数 -> 反馈增益 -> 有条件的误差保证”组织证明。

- 定理 1 是原 twist 能量证明的等价简化，让目标 gamma 直接决定阻尼。
- 定理 2 补充逐时位姿范围保证，保留原有反馈律；辅助变量 s 只用于证明。
- 定理 3 给出反馈相关模型残差下的充分设计条件。
- 能量输入是总加速度残差，位姿范围还有明确的初始状态、四元数分支和模型有效域条件。
- 文献新颖性尚未系统检索；数值核验不构成实机或完整机械臂模型认证。

## 文件

- `main.tex`：独立稿入口，使用现有中文论文样式。
- `section5-rewrite.tex`：完整推导、参数示例与贡献定位。
- `verify_original_law.py`：调用项目原有 A 与误差函数核验原控制律。
- `verification.json`：本次核验结果。
- 最终 PDF：项目根目录下 `output/pdf/gamma_design_review.pdf`。

## 复现

在项目根目录执行数值核验（需要 Python、NumPy、SciPy）：

```bash
python3 docs/gamma_design_20260921/verify_original_law.py
```

在 `texdocs` 目录执行以下命令两次（先确保输出目录存在）：

```bash
xelatex -interaction=nonstopmode -halt-on-error -output-directory ../tmp/pdfs/gamma_design_20260921 -jobname gamma_design_review ../docs/gamma_design_20260921/main.tex
```

## 已记录的核验结果

1200 次随机恒等式与辅助误差边界核验通过，恒等式最大残差约为 7.78e-15。18 次非线性误差系统积分运行未出现超出数值容差的越界；最大四元数范数偏差约为 2.23e-11。测试覆盖 5、60、150 度的允许姿态域，以及常值、旋转时变、反馈相关块不确定性。常值工况重复两次相同初值和输入，不能将 18 次运行称为 18 个不同工况。
