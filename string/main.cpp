#include <iostream>
#include <string>
#include <vector>

void parsePortConfig(const std::string &config, 
                     std::string &name, std::string &type,
                     std::vector<std::string> &elements)
{
    std::string::size_type colon = 0, start=0; 
    colon = config.find(':', start);
    if (colon == std::string::npos){
        std::cerr << "can't find the first separator in [" << config << "]" 
                  << std::endl;
        return;
    }
    name = config.substr(start, colon);
    start = colon+1;
    colon = config.find(':', start);
    if (colon == std::string::npos){
        type = config.substr(start);
        return;
    }
    std::string elist = config.substr(start, colon-start);
    std::string::size_type comma;
    start = 0;
    comma = elist.find(',', start);
    while (comma != std::string::npos){
        std::string e = elist.substr(start, comma-start);
        elements.push_back(e);
        start = comma+1;
        comma = elist.find(',', start);
    }
    elements.push_back(elist.substr(start));
    start = colon+1;
    type = config.substr(start);
}

int main(void)
{
  std::string name, type, config;
  std::vector<std::string> elements;

  config = "angle:JOINT_VALUE";
  elements.clear();
  parsePortConfig(config, name, type, elements);
  std::cout << "name = " << name << std::endl;
  std::cout << "type = " << type << std::endl;
  for (size_t i=0; i<elements.size(); i++){
    std::cout << i << ":" << elements[i] << std::endl;
  }

  config = "torque:WAIST_P,WAIST_R,CHEST,LARM_SHOULDER_P,LARM_SHOULDER_R,LARM_SHOULDER_Y,LARM_ELBOW,LARM_WRIST_Y,LARM_WRIST_P,LARM_WRIST_R,RARM_SHOULDER_P,RARM_SHOULDER_R,RARM_SHOULDER_Y,RARM_ELBOW,RARM_WRIST_Y,RARM_WRIST_P,RARM_WRIST_R:JOINT_TORQUE";
  elements.clear();
  parsePortConfig(config, name, type, elements);
  std::cout << "name = " << name << std::endl;
  std::cout << "type = " << type << std::endl;
  for (size_t i=0; i<elements.size(); i++){
    std::cout << i << ":" << elements[i] << std::endl;
  }

  return 0;
}

