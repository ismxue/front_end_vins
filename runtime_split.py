#!/usr/bin/python3

import os
import time

# 指定要监视的文件路径
file_path = 'pl_res_split.txt'

# 等待文件创建
while not os.path.exists(file_path):
    time.sleep(1)

# 初始化文件内容和上一次检查的时间戳
file_contents = []
last_check_time = time.time()
start_time=time.time()

while True:
    # 读取文件内容
    with open(file_path, 'r') as file:
        new_contents = file.readlines()

    # 如果新内容与之前的内容不同，更新上一次检查的时间戳和文件内容
    if new_contents != file_contents:
        last_check_time = time.time()
        file_contents = new_contents

    # 如果超过**秒没有新内容写入，认为已经到达最后一行
    if time.time() - last_check_time > 40:
        break

# 输出文件写入的总时间
total_time = last_check_time - start_time
print(f'Total time taken to write the file: {total_time} seconds')
