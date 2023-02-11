import pandas as pd

file_name = "Front_finalX.txt"

with open(file_name) as reader:
	s = reader.read()

s = 'answer, head_x, head_y, head_z, left_shoulder_x, left_shoulder_y, left_shoulder_z, center_x, center_y, center_z, right_shoulder_x, right_shoulder_y, right_shoulder_z, left_elbow_x, left_elbow_y, left_elbow_z, right_elbow_x, right_elbow_y, right_elbow_z, left_hand_x, left_hand_y, left_hand_z, right_hand_x, right_hand_y, right_hand_z, left_waist_x, left_waist_y, left_waist_z, right_waist_x, right_waist_y, right_waist_z, left_knees_x, left_knees_y, left_knees_z, right_knees_x, right_knees_y, right_knees_z, left_foot_x, left_foot_y, left_foot_z, right_foot_x, right_foot_y, right_foot_z \n' + s

with open(file_name, 'w') as writer:
	writer.write(s)

with open(file_name, encoding="cp932") as f:
	data_lines = f.read()

# 文字列置換
data_lines = data_lines.replace("[", "")
data_lines = data_lines.replace("]", "")

# 同じファイル名で保存
with open(file_name, mode="w", encoding="cp932") as f:
	f.write(data_lines)

#f.write('answer, head_x, head_y, head_z, left_shoulder_x, left_shoulder_y, left_shoulder_z, center_x, center_y, center_z, right_shoulder_x, right_shoulder_y, right_shoulder_z, left_elbow_x, left_elbow_y, left_elbow_z, right_elbow_x, right_elbow_y, right_elbow_z, left_hand_x, left_hand_y, left_hand_z, right_hand_x, right_hand_y, right_hand_z, left_waist_x, left_waist_y, left_waist_z, right_waist_x, right_waist_y, right_waist_z, left_knees_x, left_knees_y, left_knees_z, right_knees_x, right_knees_y, right_knees_z, left_foot_x, left_foot_y, left_foot_z, right_foot_x, right_foot_y, right_foot_z
#\n')


read_text_file = pd.read_csv (r"Front_finalX.txt")
read_text_file.to_csv (r"Front_fileX.csv", index=None)
