import shutil
from unittest import mock
import sys
import math
ball_name = "ball_pos_right2.txt"
file_name = "test_file_right3.txt"


mat = []  # 結果を入れるリスト(行列)
cnt = 0
with open(ball_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
	for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回す
		row = []  # 行のデータを保存するリスト
		num = float(line) 

		nn = 6  # 切り捨てしたい桁
		num = math.floor(num * 10 ** nn) / (10 ** nn)

		row.append(num)  # 行に保存

		mat.append(row)  # 行をnumsに保存

# print(mat)  # 結果を出力

#print(mat[0][0])
#for n in range(1,516):
#	print(mat[n][0])
# for n in range(0,516):
n=0
line_cnt =-1
# for m in range(0,mat[n][1]):
with open("test_file_right4.txt","a") as o:
	with open(file_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
		for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回
			line_cnt +=1
			if line_cnt == 70 and n < 499:
				n+=1
				line_cnt=-1
			# print(n)
			o.write("%s" % mat[n])
			o.write(", ")
			o.write("%s" % line)
				# print(mat[m][0], sep=",", file=o)
				# print(*line, file=o)


with open('test_file_right4.txt', encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open('test_file_right4.txt', mode="w", encoding="cp932") as f:
	f.write(data_lines)


print("end")