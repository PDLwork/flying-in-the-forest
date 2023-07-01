import torch

# 假设你有一个形状为 [batch_size, channels, height, width] 的张量
# 例如 [32, 34, 224, 224] 表示批量大小为32，通道数为34，高度和宽度为224的张量

# 创建一个示例张量
x = torch.randn(32, 34, 224, 224)

# 将通道维度压平
x_flat = x.view(32, -1, 448, 224)

# 打印压平后的张量维度
print(x_flat.shape)  # 输出: [32, 1, 224, 224]
