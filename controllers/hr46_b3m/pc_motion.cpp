#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
extern "C" {
#include "var.h"
#include "sq_motion.h"
}
#include "pc_motion.h"

static void set_motionbuf(tp_xp_mv_motion motionbuf[MODE2_MOTION_SIZE], std::ifstream &file){
	char str[256];
	char linebuf[1024];

	for(int frame_no=0; frame_no<MODE2_MOTION_SIZE; frame_no++){
		file.getline(linebuf, 1024);
		std::string line_str(linebuf);
		std::istringstream sstrm(line_str);
		sstrm.getline(str,10,',');
		motionbuf[frame_no].time = std::atoi(str);			//キータイム

		for(int joint_no=0;joint_no<29;joint_no++){
			sstrm.getline(str,10,',');
			motionbuf[frame_no].d[joint_no] = std::atoi(str);
		}
		for(int next=0;next<3;next++){
			sstrm.getline(str,10,',');
			motionbuf[frame_no].cntr[next] = std::atoi(str);	//次のページとオプション
		}
	}
}

void load_pc_motion(const char *dirpath)
{
	std::vector<boost::filesystem::path> file_vec;
	boost::filesystem::path dir(dirpath);
	boost::filesystem::directory_iterator end;

	for(int i=0; i < MODE2_MOTION_NUM; i++){
		for(int m=0; m < MODE2_MOTION_SIZE; m++){
			xp_mv_motionbuf_var[i][m].time = 0;
			for(int joint_no=0; joint_no < 29; joint_no++){
				xp_mv_motionbuf_var[i][m].d[joint_no] = 0;
			}
			for(int next=0; next < 3; next++){
				xp_mv_motionbuf_var[i][m].cntr[next] = 0;
			}
		}
	}
	if ( boost::filesystem::exists(dir)) {
		std::copy(boost::filesystem::directory_iterator(dir), boost::filesystem::directory_iterator(), std::back_inserter(file_vec));

		for(int i = 0; i < (int)file_vec.size(); i ++) {
			boost::filesystem::path &p = file_vec[i];
			if (boost::filesystem::is_directory(p)) {
				//ディレクトリは無視
			} else {
				std::string fleaf = p.filename().string();
				std::string ext = p.extension().string();
				int motion_no = 0;
				sscanf(fleaf.substr(0, 3).c_str(), "%d", &motion_no);
				std::cout << fleaf << ": No." << motion_no << std::endl;
				std::ifstream ifs(p.string().c_str());

				if(ext == ".txt")      set_motionbuf(xp_mv_motionbuf[motion_no], ifs);
				else if(ext == ".vm")  set_motionbuf(xp_mv_motionbuf_var[motion_no], ifs);
			}
		}
	} else {
		std::cerr << "dir not found" << std::endl;
	}
}

