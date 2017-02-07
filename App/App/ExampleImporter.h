#pragma once

#include <map>
#include <string>

#include "000_empty/000_enviroment.h"
#include "011_Arm_4_Target/011_enviroment.h"

struct ExampleImporter
{
	const char*		m_name = "";
	CommonExampleInterface::CreateFunc*  m_createFunc;

	ExampleImporter() {}

	ExampleImporter(const char* _name, CommonExampleInterface::CreateFunc* _createFunc) {
		m_name = _name;
		m_createFunc = _createFunc;
	}
};

static std::map<std::string, ExampleImporter> getExamples = {
	{ "000", ExampleImporter("000_HelloWorld", env_000) },
};