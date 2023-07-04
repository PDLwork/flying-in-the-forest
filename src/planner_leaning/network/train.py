import torch
import PlanningModle
import WeightLossFuntion

# 调整的参数
learning_rate = 0.01    #learning_rate = 1e-2
batch = 64  #设置次轮训练的个数
epoch = 10   #训练的轮数

# datalodar可能还得自己写
# 准备的训练集
# train_set = torchvision.datasets.CIFAR10(root='./dataset', train=True, download=True, transform=torchvision.transforms.ToTensor())
# 准备的测试集
# test_set = torchvision.datasets.CIFAR10(root='./dataset', train=False, download=True, transform=torchvision.transforms.ToTensor())

# 数据集的长度
# train_set_len = len(train_set)
# test_set_len = len(test_set)
# print('训练集长度为：{}'.format(train_set_len))
# print('测试集长度为：{}'.format(test_set_len))

# 用Dataloader加载数据集
# train_dataloader = torch.utils.data.DataLoader(dataset=train_set, batch_size=batch, shuffle=True, num_workers=0, drop_last=False)
# test_dataloader = torch.utils.data.DataLoader(dataset=test_set, batch_size=batch, shuffle=True, num_workers=0, drop_last=False)

# 创建网络模型
net = PlanningModle.PlannigNet()

# 构建损失函数
loss_function = WeightLossFuntion.WeightedMSELoss()

# 选择优化器，用于反向传播修正参数
optimizer = torch.optim.SGD(net.parameters(), lr=learning_rate)   #第一个变量为选择修正的参数， 第二个为学习率

total_train_step = 0    #用于记录训练的次数
total_test_step = 0     #记录测试的轮数
for i in range(epoch):
    print('----------第{}轮训练开始----------'.format(i+1))
    # 训练开始
    # net.train()   #并不是必须的,只是对特定的层有作用
    for imgs, targets in train_dataloader:
        output = net(imgs)  #输入图片求输出，相当于正向传播
        result_loss = loss_function(output, targets)  #求这次的误差

        optimizer.zero_grad()   #将梯度初始化为0，不然会累计
        result_loss.backward()  #这一步实际上是为了求梯度
        optimizer.step()    #开始修正参数

        total_train_step += 1
        if total_train_step % 100 == 0:
            print('训练次数：{}, loss:{}'.format(total_train_step, result_loss.item()))   #result_loss.item()是将tensor数据类型转换成一个数

    # 测试开始(每训练完一轮就让他跑一轮测试集)
    # net.eval()   #并不是必须的
    total_test_loss = 0     #用于记录整体测试集的误差
    total_accuracy = 0      #用于记录整体测试集正确的数目
    
    with torch.no_grad():   #关闭梯度
        for imgs, targets in test_dataloader:
            output = net(imgs)  #输出的是十分类的各个分类的概率
            result_loss = loss_function(output, targets)
            total_test_loss += result_loss.item()
            total_accuracy += (output.argmax(1) == targets).sum()   #output.argmax(1)1表示横向取最大，0表示纵向取最大，取完最大就会返回坐标，在判断是否和目标值相同，最后.sum求出正确的总个数
    
    print("整体测试集上的loss:{}，第{}轮验证的正确率为{}".format(total_test_loss, i+1, total_accuracy/test_set_len))

    # torch.save(net(), "test_net{}.path".format(i))    #这样是保存整个模型和参数  下面是只保存参数
    # torch.save(net.state_dict(), "test_net{}.path".format(i))      #保存一轮的权值
    # print('第{}轮参数模型已保存'.format(i+1))