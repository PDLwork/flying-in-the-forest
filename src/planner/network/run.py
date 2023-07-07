import random
import string
import time

def print_box(text):
    length = len(text)
    horizontal_line = '+' + '-' * (length + 40) + '+'
    empty_line = '|' + ' ' * (length + 40) + '|'

    print(horizontal_line)
    print(empty_line)
    print(f"|          {text}          |")
    print(empty_line)
    print(horizontal_line)

i = 0

while True:
    # 生成随机字符串
    random_string = ''.join(random.choice(string.ascii_letters) for _ in range(10))
    
    # 生成随机数据
    random_data = random.randint(0, 100)
    random_data1 = random.random()
    
    # 组合字符串和数据
    result = f"{random_string} {random_data}"
    
    # 输出结果
    print(f"Generated data: {result}")
    print("loss:" + str(random_data1))
    
    i += 1

    if i > 20:
        print_box("电脑跑数据中，关乎毕业，请勿触碰！！！！")
        i = 0

    # 休眠一段时间（可根据需要进行调整）
    time.sleep(0.5)
