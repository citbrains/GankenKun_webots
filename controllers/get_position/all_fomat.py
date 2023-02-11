import shutil
from unittest import mock
import math
file_name = "Front_3d_poseX.txt"
file_name_end = "Front_finalX.txt"
back_name = file_name + ".bak"

ball_file = "Ball_frontx.txt"
back_ball = ball_file + ".bak"


shutil.copy(file_name, back_name)

with open(file_name, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open(file_name, mode="w", encoding="cp932") as f:
	f.write(data_lines)

import sys

mat = []  # 結果を入れるリスト(行列)
ans = [0,0,0]
cnt = -1	  # どこのキーポイントか計測する用
with open(file_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
	for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回
		cnt += 1
		if cnt == 12:
			cnt = -1

		row = []  # 行のデータを保存するリスト
		toks = line.split(',')  # 行をカンマで分割する
		for tok in toks:  # 分割したトークン列を回す
			try:
				num = float(tok)  # 整数に変換
				nn = 6  # 切り捨てしたい桁
				num = math.floor(num * 10 ** nn) / (10 ** nn)
			except ValueError:
				print(e, file=sys.stderr)  # エラーが出たら画面に出力して
				continue  # スキップ

			row.append(num)  # 行に保存

		# row[0] = 4.5 - row[0] 

		if cnt == 1:
			memo = row

		if cnt == 2:
			memo0 = row 
			ans[0] = memo[0] + memo0[0]
			ans[1] = memo[1] + memo0[1]
			ans[2] = memo[2] + memo0[2]
			ans[0] /= 2
			ans[1] /= 2
			ans[2] /= 2
			nn = 6
			ans[0] = math.floor(ans[0] * 10 ** nn) / (10 ** nn)
			ans[1] = math.floor(ans[1] * 10 ** nn) / (10 ** nn)
			ans[2] = math.floor(ans[2] * 10 ** nn) / (10 ** nn)
			row = ans
			mat.append(row)
			mat.append(memo0)
			continue

		mat.append(row)  # 行をnumsに保存

with open(file_name, 'w') as f:
	for d in mat:
		f.write("%s\n" % d)

with open(file_name, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# print(type(data_lines))

# 同じファイル名で保存
with open(file_name, mode="w", encoding="cp932") as f:
	f.write(data_lines)
# print(mat)  # 結果を出力

################################################

import math
import numpy as np

import shutil
from unittest import mock

shutil.copy(ball_file, back_ball)

with open(ball_file, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open(ball_file, mode="w", encoding="cp932") as f:
	f.write(data_lines)

import sys

mat = []  # 結果を入れるリスト(行列)
ans = [0,0,0]
cnt = 0	  # どこのキーポイントか計測する用

with open(ball_file, 'r', encoding='utf-8') as fin:  # ファイルを開く
	for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回
		cnt += 1
		cnt_check = cnt
		cnt_check %= 2 

		row = []  # 行のデータを保存するリスト
		toks = line.split(',')  # 行をカンマで分割する
		for tok in toks:  # 分割したトークン列を回す
			try:
				num = float(tok)  # 整数に変換
			except ValueError:
				print(e, file=sys.stderr)  # エラーが出たら画面に出力して
				continue  # スキップ

			row.append(num)  # 行に保存
		#２次元空間の場合
		#角度の中心位置
		x0,y0=3.0,0
		#方向指定1
		x1,y1=4.5,0
		#方向指定2
		x2=row[0]
		y2=row[1]

		#角度計算開始
		vec1=[x1-x0,y1-y0]
		vec2=[x2-x0,y2-y0]
		absvec1=np.linalg.norm(vec1)
		absvec2=np.linalg.norm(vec2)
		inner=np.inner(vec1,vec2)
		cos_theta=inner/(absvec1*absvec2)
		theta=math.degrees(math.acos(cos_theta))
		if row[1] < 0:
			theta = theta * -1
		#print('angle='+str(round(theta,2))+'deg')
		mat.append(theta)


cnt = 0
with open(ball_file, 'w') as f:
	for d in mat:
		f.write("%s\n" % d)

################################3

import shutil
from unittest import mock
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

with open(file_name, 'w') as f:
	for d in mat:
		f.write("%s" % d)

with open(file_name, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open(file_name, mode="w", encoding="cp932") as f:
	f.write(data_lines)

######################################
import shutil
from unittest import mock
import sys
import math

mat = []  # 結果を入れるリスト(行列)
cnt = 0
with open(ball_file, 'r', encoding='utf-8') as fin:  # ファイルを開く
	for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回す
		row = []  # 行のデータを保存するリスト
		num = float(line) 

		nn = 6  # 切り捨てしたい桁
		num = math.floor(num * 10 ** nn) / (10 ** nn)

		row.append(num)  # 行に保存

		mat.append(row)  # 行をnumsに保存

n=0
line_cnt =-1
# for m in range(0,mat[n][1]):
with open(file_name_end,"w") as o:
	with open(file_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
		for line in fin.readlines():  # 行をすべて読み込んで1行ずつfor文で回
			line_cnt +=1
			if line_cnt == 185 and n < 1000:
				n+=1
				line_cnt=-1
			# print(n)
			tmp = 185-line_cnt
			o.write("%s" % mat[n])
			o.write(", ")
			o.write("%s" % tmp)
			o.write(", ")
			o.write("%s" % line)
				# print(mat[m][0], sep=",", file=o)
				# print(*line, file=o)


with open(file_name_end, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open(file_name_end, mode="w", encoding="cp932") as f:
	f.write(data_lines)

back_name_end = file_name_end + ".bak"
shutil.copy(file_name_end, back_name_end)

print("end")
