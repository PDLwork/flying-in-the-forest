import torch
import torch.nn as nn

# 假设你有一个形状为 [batch_size, channels, height, width] 的输入张量
# 例如 [32, 3, 224, 224] 表示批量大小为32，通道数为3，高度和宽度为224的张量

# 创建示例张量
x = torch.randn(32, 3, 224, 224)

# 获取通道数
channels = x.size(1)

# 假设你想对第一个通道进行全连接操作
channel_index = 0

# 将通道提取出来作为输入
channel = x[:, channel_index, :, :]

# 假设全连接层的输入大小为 224 * 224
input_size = 224 * 224

# 定义全连接层
fc = nn.Linear(input_size, 10)  # 假设输出大小为 10

# 将通道展平为 1D 张量
channel_flat = channel.view(-1, input_size)

# 进行全连接操作
output = fc(channel_flat)

# 打印输出张量的形状
print(output.shape)  # 输出: [32, 10]