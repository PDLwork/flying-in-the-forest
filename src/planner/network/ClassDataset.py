import torch
import os

# 构造数据集形式,给一个地址，然后读取文件并返回tensor形式
class PlanninglData(torch.utils.data.Dataset):
    def __init__(self, Datadir):
        self.RGB_data = modality1_data
        self.Depth_data = modality2_data
        self.LGMD_data = modality2_data
        self.State_data = modality2_data
        self.targets = targets
        
    # 数据集长度
    def __len__(self):
        return len(self.targets)
    
    # 根据索引读取数据
    def __getitem__(self, index):
        modality1_sample = self.modality1_data[index]
        modality2_sample = self.modality2_data[index]
        target = self.targets[index]
        
        return modality1_sample, modality2_sample, target

if __name__ == '__main__':
    # Example usage
    modality1_data = torch.randn(100, 10)  # 模态1的数据，大小为100x10
    modality2_data = torch.randn(100, 20)  # 模态2的数据，大小为100x20
    targets = torch.randint(0, 2, (100,))  # 目标标签，大小为100

    dataset = PlanninglData(modality1_data, modality2_data, targets)

    # 访问数据集样本
    sample_modality1, sample_modality2, sample_target = dataset[20]

    print(sample_modality1)
    print(type(sample_modality1))

    print(sample_modality2)
    print(sample_target)