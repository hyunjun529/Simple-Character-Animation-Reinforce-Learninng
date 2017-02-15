#pragma once

#include <iostream>
#include <fstream>
#include <string>

class Logger {
public:
	
	std::ofstream fout;
	
	std::string path_default = "logs/";
	std::string path_file = "default_log.csv";

	Logger() {}
	~Logger() {}

	void fs_open() {
		fout.open(path_default + path_file, std::ofstream::out);
	}

	void fs_open(const char* _file_name) {
		path_file = _file_name;
		fs_open();
	}

	void fs_close() {
		fout.close();
	}
};