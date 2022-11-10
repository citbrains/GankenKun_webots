import math
import numpy as np

import shutil
from unittest import mock
file_name = "ball_pos_right.txt"
back_name = file_name + ".bak"
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
cnt = 0	  # どこのキーポイントか計測する用

with open(file_name, 'r', encoding='utf-8') as fin:  # ファイルを開く
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
with open('ball_pos_right2.txt', 'a') as f:
	for d in mat:
		f.write("%s\n" % d)
		

#output=""

#with open('comp_ball_xxx.txt', encoding="utf-8") as f:
##    for line in f:
 #       if not line.isspace():
#            output+=line

#f = open(r"comp_ball_no_space_xxx.txt","w")
#f.write(output)


# with open('comp_ball.txt', encoding="cp932") as f:
# 	data_lines = f.read()
#
# # 文字列置換
# data_lines = data_lines.replace("[", "")
# data_lines = data_lines.replace("]", "")
#
# # print(type(data_lines))
#
# # 同じファイル名で保存
# with open('comp_ball.txt', mode="w", encoding="cp932") as f:
# 	f.write(data_lines)
# # print(mat)  # 結果を出力