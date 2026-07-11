# Day3 模型文件说明

本目录需要放置 LeNet ONNX 模型和标签文件，从 26_SP 仓库复制：

```bash
# 在 Day3-Traditional 目录下执行：
cp ../../Horizon_Rm_Vision_26/assets/lenet.onnx assets/
cp ../../Horizon_Rm_Vision_26/assets/label.txt  assets/
```

文件说明：
- `lenet.onnx` — LeNet 数字分类 ONNX 模型（用于装甲板数字识别）
- `label.txt`  — 类别标签文件（每行一个类别名：negative, 1, 2, 3, 4, 5, outpost, sentry, base）

如果不需要数字分类功能（选做），可以跳过此步骤，
MyTraditionalDetector 构造函数中传入空的 model_path 即可。
