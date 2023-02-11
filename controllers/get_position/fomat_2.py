import shutil
from unittest import mock
file_name = "3d_pose.txt"
import sys

mat = []  # 結果を入れるリスト(行列)
ans = [0,0,0]
cnt = -1	  # どこのキーポイントか計測する用
with open(file_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
	for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回
		cnt += 1
		if cnt == 13:
			cnt = -1

		else:
			line = line.replace("\n", ", ")

		mat.append(line)  # 行をnumsに保存

with open('3d_pose_2.txt', 'a') as f:
	for d in mat:
		f.write("%s" % d)

with open('3d_pose_2.txt', encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open('3d_pose_2.txt', mode="w", encoding="cp932") as f:
	f.write(data_lines)

