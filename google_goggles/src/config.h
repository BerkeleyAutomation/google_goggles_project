#ifndef BENK_CONFIG
#define BENK_CONFIG

#ifndef BENK_STRINGIFY
#define BENK_STRINGIFY(X) BENK_STRINGIFY_HELPER(X)
#define BENK_STRINGIFY_HELPER(X) #X
#endif


/* Modified from code I got from John Schulman */

/*
struct MyConfig : public benk::Config {
	float param1;
	std::string param2_has_default;
	int param3;

	MyConfig() : benk::Config() {
		addOption(param1,float);
		addOption(param2_has_default,std::string,"thedefault");
		addOptions(param3,"v,param-number-three",int);
	}
};

namespace ::Config {
static PoseEstimatorConfig PoseEstimator = PoseEstimatorConfig();
}

//Use Config::PoseEstimator.param1
----

benk::Parser parser;
parser.addGroup(Config::PoseEstimator);
parser.read(argc, argv);

float p = Config::PoseEstimator.param1
*/

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

namespace benk {

struct ParameterBase {
	std::string m_name;
	std::string m_desc;
	virtual void addToBoost(po::options_description&) = 0;
	virtual ~ParameterBase() {}
};

template <typename T>
struct Parameter : ParameterBase {
	T* m_value;
	
	Parameter(std::string name, T* value, std::string desc,T default_value=T()) {
		m_name = name;
		m_value = value;
		m_desc = desc;
		*m_value = default_value;
	}
	void addToBoost(po::options_description& od) {
		std::cout << "name " << m_name << " value " << *m_value << std::endl;
		od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value), m_desc.c_str());
	}
};

struct Config {
	std::vector<ParameterBase*> params;
	
	#define addOption(field, type, ...) addParam<type>(benk::Config::underscore_to_hyphen(BENK_STRINGIFY(field)),&(this->field), ##__VA_ARGS__)
	#define addOptions(field, other_names, type, ...) addParam<type>(benk::Config::underscore_to_hyphen(BENK_STRINGIFY(field)),other_names, &(this->field), ##__VA_ARGS__)
	#define addOptionWithHelp(field, type, help, ...) addParamWithHelp<type>(benk::Config::underscore_to_hyphen(BENK_STRINGIFY(field)),&(this->field), help, ##__VA_ARGS__)
	#define addOptionsWithHelp(field, other_names, type, help, ...) addParamWithHelp<type>(benk::Config::underscore_to_hyphen(BENK_STRINGIFY(field)),other_names, &(this->field), help, ##__VA_ARGS__)
	
	template<typename T>
	void addParam(const std::string& name,T* value,T default_value=T()) {
		params.push_back(new Parameter<T>(name, value, "", default_value));
		std::cout << "adding param " << name << " " << params.size() << std::endl;
	}
	
	template<typename T>
	void addParam(const std::string& name,const std::string& other_names, T* value,T default_value=T()) {
		params.push_back(new Parameter<T>(name + "," + other_names, value, "", default_value));
	}
	
	template<typename T>
	void addParamWithHelp(const std::string& name,T*value,const std::string& help,T default_value=T()) {
		params.push_back(new Parameter<T>(name, value, help, default_value));
	}
	
	template<typename T>
	void addParamWithHelp(const std::string& name,const std::string& other_names,T*value,const std::string& help,T default_value=T()) {
		params.push_back(new Parameter<T>(name + "," + other_names, value, help, default_value));
	}

	static std::string underscore_to_hyphen(std::string val) {
		for (size_t i=0;i<val.size();i++) {
			if (val[i] == '_') {
				val[i] = '-';
			}
		}
		return val;
	}
};

class Parser {
	std::vector<Config*> m_configs;
	po::positional_options_description m_positional_args;
	po::variables_map m_vm;
	bool m_read;
public:
	Parser() : m_read(false) { }
	
	void addGroup(Config& config) {
		m_configs.push_back(&config);
		std::cout << "added group" << std::endl;
	}
	void addGroup(Config* config) {
		m_configs.push_back(config);
		std::cout << "added group*" << std::endl;
	}
	void addArg(const std::string& name,int max_count=1) {
		m_positional_args.add(name.c_str(),max_count);
	}

	void read(int argc, char* argv[]) {
		// create boost options_description based on variables, parser
		po::options_description od;
		od.add_options()("help,h", "produce help message");
		BOOST_FOREACH(Config* config, m_configs) {
			std::cout << "1 config" << std::endl;
			BOOST_FOREACH(ParameterBase* param, config->params) {
				std::cout << "adding to boost" << std::endl;
				param->addToBoost(od);
			}
		}
		//po::variables_map vm;   
		po::store(
				po::command_line_parser(argc, argv).options(od).positional(m_positional_args).run(),
				m_vm);
		if (m_vm.count("help")) {
			std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
			std::cout << od << std::endl;
			exit(0);
		}
		po::notify(m_vm);
		m_read = true;
	}
	
	template <typename T>
	T getOption(const std::string& name) {
		return m_vm[name].as< T >();
	}
	
	template <typename T>
	T getArg(int pos) {
		return m_vm[m_positional_args.name_for_position(pos)].as< T >();
	}
};

template<typename T>
std::vector<T> parseVector(const std::string& str) {
	std::vector<T> v;
	std::stringstream ss(str);
	while (!ss.eof()) {
		T value;
		ss >> value;
		v.push_back(value);
		if (ss.fail()) {
			v.clear();
			std::cerr << "Could not parse " << value << " from " << str << " into " << typeid(T) << std::endl;
			break;
		}
	}
	return v;
}

}

#endif
