import torch
import torchvision

class Mynet(torch.nn.Module):
    def __init__(self):
        super().__init__()

        unknow_channel = 32

        self.LGMD_Backbone = torch.nn.Sequential(
            # Conv1d可以理解为一个长方形的2d卷积
            # 输入320*240的图像
            torch.nn.Conv1d(in_channels=320, out_channels=128, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=128, out_channels=64, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=64, out_channels=64, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=64, out_channels=32, kernel_size=2, stride=1),
            torch.nn.LeakyReLU()
        )

        self.Depth_Backbone = torch.nn.Sequential(
            torch.nn.Conv1d(in_channels=320, out_channels=128, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=128, out_channels=64, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=64, out_channels=64, kernel_size=2, stride=1),
            torch.nn.LeakyReLU(),
            torch.nn.Conv1d(in_channels=64, out_channels=32, kernel_size=2, stride=1),
            torch.nn.LeakyReLU()
        )

        self.State_Backbone = torch.nn.Sequential(
            torch.nn.Conv1d(in_channels=1, out_channels=64, kernel_size=3, stride=1, padding=1),
            torch.nn.LeakyReLU(negative_slope=0.5),
            torch.nn.Conv1d(in_channels=64, out_channels=32, kernel_size=3, stride=1, padding=1),
            torch.nn.LeakyReLU(negative_slope=0.5),
            torch.nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=1, padding=1),
            torch.nn.LeakyReLU(negative_slope=0.5),
            torch.nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=1, padding=1),
            # 这一步还有点疑问 要不要留呢？
            torch.nn.Conv1d(in_channels=32, out_channels=unknow_channel, kernel_size=3, stride=1, padding=1)
        )



    def forward(self, input_LGMD, input_Depth, input_State):
        LGMD_output = self.LGMD_Backbone(input_LGMD)

        Depth_output = self.Depth_Backbone(input_Depth)

        State_output = self.State_Backbone(input_State)
        # 原文有换通道顺序的
        # State_output = State_output.permute(0,2,1)

        return LGMD_output, Depth_output, State_output

# 一般在这里验证网络的正确性
if __name__ == "__main__":
    model = Mynet()
    # print(model)

    input1 = torch.rand((64, 320, 240))
    input2 = torch.rand((64, 320, 240))
    input3 = torch.rand((64, 1, 20))

    LGMD_output, Depth_output, State_output = model(input1, input2, input3)
    print(LGMD_output.shape, Depth_output.shape, State_output.shape)


    # large = torchvision.models.mobilenet_v3_large(pretrained=True, width_mult=1.0,  reduced_tail=False, dilated=False)
    # small = torchvision.models.mobilenet_v3_small(pretrained=True)
    # quantized = torchvision.models.quantization.mobilenet_v3_large(pretrained=True)
