import shutil
file_name = "test_file.txt"
back_name = file_name + ".bak"
shutil.copy(file_name, back_name)

with open(file_name, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# print(type(data_lines))

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
		if cnt == 14:
			cnt = -1

		row = []  # 行のデータを保存するリスト
		toks = line.split(',')  # 行をカンマで分割する
		for tok in toks:  # 分割したトークン列を回す
			try:
				num = float(tok)  # 整数に変換
			except ValueError:
				print(e, file=sys.stderr)  # エラーが出たら画面に出力して
				continue  # スキップ

			row.append(num)  # 行に保存

		if cnt == 3 or cnt == 5:
			memo1 = row

		if cnt == 4 or cnt ==6:
			memo2 = row
			ans[0] = memo1[0] + memo2[0]
			ans[1] = memo1[1] + memo2[1]
			ans[2] = memo1[2] + memo2[2]
			ans[0] /= 2
			ans[1] /= 2
			ans[2] /= 2
			row = ans
			# print(row)

		mat.append(row)  # 行をnumsに保存
with open('comp_file.txt', 'a') as f:
	for d in mat:
		f.write("%s\n" % d)

with open('comp_file.txt', encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# print(type(data_lines))

# 同じファイル名で保存
with open('comp_file.txt', mode="w", encoding="cp932") as f:
	f.write(data_lines)
# print(mat)  # 結果を出力
