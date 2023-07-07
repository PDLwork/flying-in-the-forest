import torch

# 构造带权重损失函数
class WeightedMSELoss(torch.nn.Module):
    def __init__(self):
        super(WeightedMSELoss, self).__init__()
        self.weights = torch.tensor([1.0, 1.0, 1.0, 0.9, 0.9, 0.9, 0.8, 0.8, 0.8, 0.7, 0.7, 0.7, 0.6, 0.6, 0.6,
                                     0.5, 0.5, 0.5, 0.4, 0.4, 0.4, 0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.1])
        
    def forward(self, input, target):
        batch_size = input.size(0)
        
        # Compute weighted squared difference
        weighted_diff = self.weights * (input - target) ** 2
        
        # Compute the sum of weighted squared differences
        loss = torch.sum(weighted_diff) / batch_size
        
        return loss

# 验证网络的正确性
if __name__ == '__main__':
    pass