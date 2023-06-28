import tensorflow as tf

# 定义输入张量
input_tensor = tf.random.normal((1, 10, 3))  # 假设输入尺寸为 [batch_size, length, channels]

# 定义一维卷积层（包含偏置）
conv = tf.keras.layers.Conv1D(filters=16, kernel_size=3)

# 进行卷积操作
output_tensor = conv(input_tensor)

print(output_tensor.shape)
