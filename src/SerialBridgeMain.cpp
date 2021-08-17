#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <vector>
#include <queue>
#include <stack>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <iostream>

#include "amm_std.h"
#include "Errors.h"

extern "C" {
#include "Serial/arduino-serial-lib.h"
}

#include "tinyxml2.h"

#define DEFAULT_SERIAL_DEVICE "/dev/ttyMSM1"
#define DEFAULT_BAUD_RATE 115200
const std::string SERIAL_BRIDGE_VERSION = "1.0.0";

int g_log_level = 0;

using namespace AMM;
using namespace std::chrono;
using namespace tinyxml2;

bool first_message = true;
bool closed = false;
bool initializing = true;

std::string globalInboundBuffer;
std::string requestPrefix = "[REQUEST]";
std::string reportPrefix = "[REPORT]";
std::string actionPrefix = "[AMM_Command]";
std::string genericTopicPrefix = "[";
std::string xmlPrefix = "<?xml";

const std::string capabilityPrefix = "CAPABILITY=";
const std::string settingsPrefix = "SETTINGS=";
const std::string statusPrefix = "STATUS=";
const std::string configPrefix = "CONFIG=";
const std::string modulePrefix = "MODULE_NAME=";
const std::string registerPrefix = "REGISTER=";
const std::string keepHistoryPrefix = "KEEP_HISTORY=";
const std::string keepAlivePrefix = "[KEEPALIVE]";
const std::string loadScenarioPrefix = "LOAD_SCENARIO:";
const std::string haltingString = "HALTING_ERROR";
const std::string sysPrefix = "[SYS]";
const std::string actPrefix = "[ACT]";
const std::string loadPrefix = "LOAD_STATE:";

std::vector<std::string> subscribedTopics;
std::vector<std::string> publishedTopics;
std::map<std::string, std::string> subMaps;
std::map<std::string, std::map<std::string, std::string>> equipmentSettings;

std::queue<std::string> transmitQ;

int fd = -1;
int rc;


// Boost doesn't offer any obvious way to construct a usage string
// from an infinite list of positional parameters.  This hack
// should work in most reasonable cases.
std::vector<std::string> get_unlimited_positional_args_(const boost::program_options::positional_options_description& p)
{
  assert(p.max_total_count() == std::numeric_limits<unsigned>::max());

  std::vector<std::string> parts;

  // reasonable upper limit for number of positional options:
  const int MAX = 1000;
  std::string last = p.name_for_position(MAX);

  for (int i = 0; true; ++i) {
    std::string cur = p.name_for_position(i);
    if (cur == last) {
      parts.push_back(cur);
      return parts;
    }
    parts.push_back(cur);
  }
  return parts; // never get here
}

std::string make_usage_string_(const std::string& program_name, const boost::program_options::options_description& desc, boost::program_options::positional_options_description& p)
{
  std::vector<std::string> parts;
  parts.push_back("Usage: ");
  parts.push_back(program_name);
  if (desc.options().size() > 0) {
    parts.push_back("[options]");
  }
  size_t N = p.max_total_count();
  if (N == std::numeric_limits<unsigned>::max()) {
    std::vector<std::string> args = get_unlimited_positional_args_(p);
    parts.insert(parts.end(), args.begin(), args.end());
  } else {
    for (int i = 0; i < N; ++i) {
      parts.push_back(p.name_for_position(i));
    }
  }
  std::ostringstream oss;
  std::copy(
    parts.begin(),
    parts.end(),
    std::ostream_iterator<std::string>(oss, " "));
  oss << '\n'
      << desc;
  return oss.str();
}


void sendConfigInfo(std::string scene, std::string module) {
   std::ostringstream static_filename;
   static_filename << "static/module_configuration_static/" << scene << "_" << module << ".txt";
   LOG_DEBUG << "Loading config from filename: " << static_filename.str();
   std::ifstream ifs(static_filename.str());
   std::string configContent((std::istreambuf_iterator<char>(ifs)),
                             (std::istreambuf_iterator<char>()));
   ifs.close();
   if (configContent.empty()) {
      LOG_ERROR << "Configuration empty.";
      return;
   }
   std::vector<std::string> v = Utility::explode("\n", configContent);
   for (int i = 0; i < v.size(); i++) {
      std::string rsp = v[i] + "\n";
      transmitQ.push(rsp);
   }
};

class AMMListener : public ListenerInterface {
public:
    void onNewPhysiologyWaveform(AMM::PhysiologyWaveform &n, SampleInfo_t *info) {
       std::string hfname = "HF_" + n.name();
       if (std::find(subscribedTopics.begin(), subscribedTopics.end(), hfname) != subscribedTopics.end()) {
          std::ostringstream messageOut;
	  std::map<std::string, std::string>::iterator i = subMaps.find(hfname);
          if (i == subMaps.end()) {
             messageOut << "[AMM_Node_Data]" << n.name() << "=" << n.value() << std::endl;
          } else {
             messageOut << "[" << i->first << "]" << n.value() << std::endl;
          }

	  if(g_log_level > 0){
             LOG_INFO << messageOut.str();
	  }
          rc = serialport_write(fd, messageOut.str().c_str());
          if (rc == -1) {
             LOG_ERROR << " Error writing to serial port";
          }
       }
    }

    void onNewPhysiologyValue(AMM::PhysiologyValue &n, SampleInfo_t *info) {
       // Publish values that are supposed to go out on every change
       if (std::find(subscribedTopics.begin(), subscribedTopics.end(), n.name()) != subscribedTopics.end()) {
          std::ostringstream messageOut;
	  std::map<std::string, std::string>::iterator i = subMaps.find(n.name());
          if (i == subMaps.end()) {
             messageOut << "[AMM_Node_Data]" << n.name() << "=" << n.value() << std::endl;
          } else {
             messageOut << "[" << i->first << "]" << n.value() << std::endl;
          }

	  if(g_log_level > 0){
             LOG_INFO << messageOut.str();
	  }
          rc = serialport_write(fd, messageOut.str().c_str());
          if (rc == -1) {
             LOG_ERROR << " Error writing to serial port";
          }
       }
    }

    void onNewPhysiologyModification(AMM::PhysiologyModification &pm, SampleInfo_t *info) {
       // Publish values that are supposed to go out on every change
       std::ostringstream messageOut;
       messageOut << "[AMM_Physiology_Modification]"
                  << "type=" << pm.type() << ";"
                  //<< "location=" << pm.location().description() << ";"
                  //<< "learner_id=" << pm.practitioner() << ";"
                  << "payload=" << pm.data()
                  << std::endl;
       std::string stringOut = messageOut.str();
       LOG_DEBUG << "Physiology modification received from AMM: " << stringOut;

       if (std::find(subscribedTopics.begin(), subscribedTopics.end(), pm.type()) != subscribedTopics.end() ||
           std::find(subscribedTopics.begin(), subscribedTopics.end(), "AMM_Physiology_Modification") !=
           subscribedTopics.end()
          ) {
          transmitQ.push(messageOut.str());
       }
    }

    void onNewRenderModification(AMM::RenderModification &rendMod, SampleInfo_t *info) {
       // Publish values that are supposed to go out on every change
       std::ostringstream messageOut;
       messageOut << "[AMM_Render_Modification]"
                  << "type=" << rendMod.type() << ";"
                  //<< "location=" << rm.location().description() << ";"
                  //<< "learner_id=" << rm.practitioner() << ";"
                  << "payload=" << rendMod.data()
                  << std::endl;
       std::string stringOut = messageOut.str();

       LOG_DEBUG << "Render modification received from AMM: " << stringOut;

       if (std::find(subscribedTopics.begin(), subscribedTopics.end(), rendMod.type()) != subscribedTopics.end() ||
           std::find(subscribedTopics.begin(), subscribedTopics.end(), "AMM_Render_Modification") !=
           subscribedTopics.end()
          ) {
          transmitQ.push(messageOut.str());
       }

    }
    void onNewSimulationControl(AMM::SimulationControl &simControl, SampleInfo_t *info) {

       switch (simControl.type()) {
          case AMM::ControlType::RUN: {
             LOG_INFO << "SimControl Message recieved; Run sim.";
             std::ostringstream cmdMessage;
             cmdMessage << "[AMM_Command]START_SIM\n";
             transmitQ.push(cmdMessage.str());
             break;
          }

          case AMM::ControlType::HALT: {
             LOG_INFO << "SimControl recieved; Halt sim";
             std::ostringstream cmdMessage;
             cmdMessage << "[AMM_Command]PAUSE_SIM\n";
             transmitQ.push(cmdMessage.str());
             break;
          }

          case AMM::ControlType::RESET: {
             LOG_INFO << "SimControl recieved; Reset sim";
             std::ostringstream cmdMessage;
             cmdMessage << "[AMM_Command]RESET_SIM\n";
             transmitQ.push(cmdMessage.str());
             break;
          }

          case AMM::ControlType::SAVE: {
             LOG_INFO << "SimControl recieved; Save sim";
             std::ostringstream cmdMessage;
             cmdMessage << "[AMM_Command]SAVE_STATE\n";
             transmitQ.push(cmdMessage.str());
             break;
          }
       }
    }

    void onNewCommand(AMM::Command &c, eprosima::fastrtps::SampleInfo_t *info) {
       LOG_DEBUG << "Command received from AMM: " << c.message();
       if (!c.message().compare(0, sysPrefix.size(), sysPrefix)) {
          std::string value = c.message().substr(sysPrefix.size());
          // Send it on through the bridge
          std::ostringstream cmdMessage;
          cmdMessage << "[AMM_Command]" << value << "\n";
          transmitQ.push(cmdMessage.str());
       } else {
          std::ostringstream cmdMessage;
          cmdMessage << "[AMM_Command]" << c.message() << "\n";
          transmitQ.push(cmdMessage.str());
       }
    }
};

const std::string moduleName = "AMM_Serial_Bridge";
const std::string configFile = "config/serial_bridge_amm.xml";
AMM::DDSManager<AMMListener> *g_mgr;
AMM::UUID m_uuid;

void PublishSettings(std::string const &equipmentType) {
   std::ostringstream payload;
   LOG_INFO << "Publishing equipment " << equipmentType << " settings";
   for (auto &inner_map_pair : equipmentSettings[equipmentType]) {
      payload << inner_map_pair.first << "=" << inner_map_pair.second
              << std::endl;
      LOG_DEBUG << "\t" << inner_map_pair.first << ": " << inner_map_pair.second;
   }

   AMM::InstrumentData i;
   i.instrument(equipmentType);
   i.payload(payload.str());
   g_mgr->WriteInstrumentData(i);
}

void readHandler() {
   std::vector<std::string> v = Utility::explode("\n", globalInboundBuffer);
   globalInboundBuffer.clear();
   for (int i = 0; i < v.size(); i++) {
      std::string rsp = v[i];
      if (!rsp.compare(0, reportPrefix.size(), reportPrefix)) {
         std::string value = rsp.substr(reportPrefix.size());
         LOG_DEBUG << "Received report via serial: " << value;
      } else if (!rsp.compare(0, actionPrefix.size(), actionPrefix)) {
         std::string value = rsp.substr(actionPrefix.size());
         LOG_INFO << "Received command via serial, publishing to AMM: " << value;
         AMM::Command cmdInstance;
         boost::trim_right(value);
         cmdInstance.message(value);
         g_mgr->WriteCommand(cmdInstance);
      } else if (!rsp.compare(0, xmlPrefix.size(), xmlPrefix)) {
         std::string value = rsp;
         LOG_INFO << "Received XML via serial";
         LOG_DEBUG << "\tXML: " << value;
         tinyxml2::XMLDocument doc(false);
         doc.Parse(value.c_str());
         tinyxml2::XMLNode *root = doc.FirstChildElement("AMMModuleConfiguration");

         if (root) {
            tinyxml2::XMLNode *mod = root->FirstChildElement("module");
            tinyxml2::XMLElement *module = mod->ToElement();

            if (initializing) {
               LOG_INFO << "Module is initializing, so we'll publish the Operational Description.";

               std::string module_name = module->Attribute("name");
               std::string manufacturer = module->Attribute("manufacturer");
               std::string model = module->Attribute("model");
               std::string serial_number = module->Attribute("serial_number");
               std::string module_version = module->Attribute("module_version");

               AMM::OperationalDescription od;
               od.name(module_name);
               od.model(module_name);
               od.manufacturer(manufacturer);
               od.serial_number(serial_number);
               od.module_id(m_uuid);
               od.module_version(module_version);
               // const std::string capabilities = AMM::Utility::read_file_to_string("config/tcp_bridge_capabilities.xml");
               // od.capabilities_schema(capabilities);
               g_mgr->WriteOperationalDescription(od);

               initializing = false;
            }


            tinyxml2::XMLNode *caps = mod->FirstChildElement("capabilities");

            if (caps) {
               // Clear the subs and pubs before we re-gather them

               bool firstSub = true;
               bool firstPub = true;

               for (tinyxml2::XMLNode *node = caps->FirstChildElement(
                  "capability"); node; node = node->NextSibling()) {
                  tinyxml2::XMLElement *cap = node->ToElement();
                  std::string capabilityName = cap->Attribute("name");

                  tinyxml2::XMLElement *starting_settings = cap->FirstChildElement(
                     "starting_settings");
                  if (starting_settings) {
                     LOG_DEBUG << "Received starting settings";
                     for (tinyxml2::XMLNode *settingNode = starting_settings->FirstChildElement(
                        "setting"); settingNode; settingNode = settingNode->NextSibling()) {
                        tinyxml2::XMLElement *setting = settingNode->ToElement();
                        std::string settingName = setting->Attribute("name");
                        std::string settingValue = setting->Attribute("value");
                        LOG_DEBUG << "[" << settingName << "] = " << settingValue;
                     }
                  }

                  tinyxml2::XMLElement *configEl =
                     cap->FirstChildElement("configuration");
                  if (configEl) {
                     for (tinyxml2::XMLNode *settingNode =
                        configEl->FirstChildElement("setting");
                          settingNode; settingNode = settingNode->NextSibling()) {
                        tinyxml2::XMLElement *setting = settingNode->ToElement();
                        std::string settingName = setting->Attribute("name");
                        std::string settingValue = setting->Attribute("value");
                        equipmentSettings[capabilityName][settingName] =
                           settingValue;
                     }
                     PublishSettings(capabilityName);
                  }

                  // Store subscribed topics for this capability
                  tinyxml2::XMLNode *subs = node->FirstChildElement("subscribed_topics");
                  if (subs) {
                     if (firstSub) {
                        subscribedTopics.clear();
                        firstSub = false;
                     }
                     for (tinyxml2::XMLNode *sub = subs->FirstChildElement(
                        "topic"); sub; sub = sub->NextSibling()) {
                        tinyxml2::XMLElement *s = sub->ToElement();
                        std::string subTopicName = s->Attribute("name");


                        if (s->Attribute("nodepath")) {
                           std::string subname = s->Attribute("nodepath");
                           if (subTopicName == "AMM_HighFrequencyNode_Data") {
                              subTopicName = "HF_" + subname;
                           } else {
                              subTopicName = subname;
                           }
                        }

                        if (s->Attribute("map_name")) {
                           std::string subMapName = s->Attribute("map_name");
                           subMaps[subTopicName] = subMapName;
                        }

                        Utility::add_once(subscribedTopics, subTopicName);
                        LOG_DEBUG << "[" << capabilityName << "][SUBSCRIBE]" << subTopicName;
                     }
                  }

                  // Store published topics for this capability
                  tinyxml2::XMLNode *pubs = node->FirstChildElement("published_topics");
                  if (pubs) {
                     if (firstPub) {
                        publishedTopics.clear();
                        firstPub = false;
                     }

                     for (tinyxml2::XMLNode *pub = pubs->FirstChildElement(
                        "topic"); pub; pub = pub->NextSibling()) {
                        tinyxml2::XMLElement *p = pub->ToElement();
                        std::string pubTopicName = p->Attribute("name");
                        Utility::add_once(publishedTopics, pubTopicName);
                        LOG_DEBUG << "[" << capabilityName << "][PUBLISH]" << pubTopicName;
                     }
                  }
               }
            }
         } else {
            tinyxml2::XMLNode *root = doc.FirstChildElement("AMMModuleStatus");
            tinyxml2::XMLElement *module = root->FirstChildElement("module")->ToElement();
            const char *name = module->Attribute("name");
            std::string nodeName(name);

            tinyxml2::XMLElement *caps = module->FirstChildElement("capabilities");
            if (caps) {
               for (tinyxml2::XMLNode *node = caps->FirstChildElement(
                  "capability"); node; node = node->NextSibling()) {
                  tinyxml2::XMLElement *cap = node->ToElement();
                  std::string capabilityName = cap->Attribute("name");
                  std::string statusVal = cap->Attribute("status");

                  AMM::Status s;
                  s.module_id(m_uuid);
                  s.module_name(nodeName);
                  s.capability(capabilityName);

                  if (statusVal == "OPERATIONAL") {
                     s.value(AMM::StatusValue::OPERATIONAL);
                  } else if (statusVal == "HALTING_ERROR") {
                     s.value(AMM::StatusValue::INOPERATIVE);
                     if (cap->Attribute("message")) {
                        std::string errorMessage = cap->Attribute("message");
                        s.message(errorMessage);
                     } else {
                     }
                  } else if (statusVal == "IMPENDING_ERROR") {
                     s.value(AMM::StatusValue::EXIGENT);
                     if (cap->Attribute("message")) {
                        std::string errorMessage = cap->Attribute("message");
                        s.message(errorMessage);
                     } else {

                     }
                  } else {
                     LOG_ERROR << "Invalid status value " << statusVal << " for capability " << capabilityName;
                  }
                  g_mgr->WriteStatus(s);
               }
            }
         }
      } else if (!rsp.compare(0, genericTopicPrefix.size(), genericTopicPrefix)) {
         std::string topic, message, modType, modLocation, modPayload, modInfo;
         unsigned first = rsp.find("[");
         unsigned last = rsp.find("]");
         topic = rsp.substr(first + 1, last - first - 1);
         message = rsp.substr(last + 1);

         std::list<std::string> tokenList;
         split(tokenList, message, boost::algorithm::is_any_of(";"), boost::token_compress_on);
         std::map<std::string, std::string> kvp;

         BOOST_FOREACH(std::string token, tokenList) {
            size_t sep_pos = token.find_first_of("=");
            std::string key = token.substr(0, sep_pos);
            std::string value = (sep_pos == std::string::npos ? "" : token.substr(sep_pos + 1,
                                                                                  std::string::npos));
            kvp[key] = value;
            if (key == "type") {
               modType = kvp[key];
            } else if (key == "location") {
               modLocation = kvp[key];
            } else if (key == "info") {
               modInfo = kvp[key];
            } else if (key == "payload") {
               modPayload = kvp[key];
            }

         }

         if (topic == "AMM_Render_Modification") {
            AMM::RenderModification renderMod;
            renderMod.type(modType);
            renderMod.data(modPayload);
            //renderMod.location().description(modLocation);
            g_mgr->WriteRenderModification(renderMod);
         } else if (topic == "AMM_Physiology_Modification") {
            AMM::PhysiologyModification physMod;
            physMod.type(modType);
            physMod.data(modPayload);
            //physMod.location().description(modLocation);
            g_mgr->WritePhysiologyModification(physMod);
         } else if (topic == "AMM_Performance_Assessment") {
            AMM::Assessment assessment;
            assessment.comment(modInfo);
            g_mgr->WriteAssessment(assessment);
         } else if (topic == "AMM_Diagnostics_Log_Record") {
            if (modType == "info") {
               LOG_INFO << modPayload;
            } else if (modType == "warning") {
               LOG_WARNING << modPayload;
            } else if (modType == "error") {
               LOG_ERROR << modPayload;
            } else {
               LOG_DEBUG << modPayload;
            }
         } else {
            LOG_DEBUG << "Unknown topic: " << topic;
         }
      } else {
         if (!rsp.empty() && rsp != "\r") {
            LOG_DEBUG << "Serial debug: " << rsp;
         }
      }
   }
}





void checkForExit() {
   std::string action;
   while (!closed) {
      getline(std::cin, action);
      std::transform(action.begin(), action.end(), action.begin(), ::toupper);
      if (action == "EXIT") {
         closed = true;
         LOG_INFO << "Shutting down.";
      }
   }
}


void signalHandler(int signum) {
   LOG_WARNING << "Interrupt signal (" << signum << ") received.";

   if (signum == 15) {
      serialport_close(fd);
      LOG_INFO << "Shutdown complete";
   }

   exit(signum);
}


void PublishOperationalDescription() {
   AMM::OperationalDescription od;
   od.name(moduleName);
   od.model("Serial Bridge");
   od.manufacturer("Vcom3D");
   od.serial_number("1.0.0");
   od.module_id(m_uuid);
   od.module_version("1.0.0");
   const std::string capabilities = AMM::Utility::read_file_to_string("config/serial_bridge_capabilities.xml");
   od.capabilities_schema(capabilities);
   od.description();
   g_mgr->WriteOperationalDescription(od);
}

void PublishConfiguration() {
   AMM::ModuleConfiguration mc;
   auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
   mc.timestamp(ms);
   mc.module_id(m_uuid);
   mc.name(moduleName);
   const std::string configuration = AMM::Utility::read_file_to_string("config/serial_bridge_configuration.xml");
   mc.capabilities_configuration(configuration);
   g_mgr->WriteModuleConfiguration(mc);
}


int main(int argc, char *argv[]) {
   static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
   plog::init(plog::verbose, &consoleAppender);

   using namespace boost::program_options;

   std::string help_message;

   options_description desc("Allowed options");
   options_description all_options;
   positional_options_description p;
   variables_map vm;
   
   // Declare the supported options.
   desc.add_options() /**/
       ("help,h"  , "produce help message") /**/
       ("device,p"   , value<std::string>()->default_value(DEFAULT_SERIAL_DEVICE), "Change the default device used for serial communication.")/**/
       ("rate,b"     , value<int>()->default_value(static_cast<int>(DEFAULT_BAUD_RATE)), "Set the baud rate for communication") /**/
       ("verbosity,v", value<int>()->default_value(static_cast<int>(0)), "Adjust the default verbosity rate") /**/
       ("version" , bool_switch()->default_value(false), "Print the version of this application");

   all_options.add(desc);
    
   boost::filesystem::path exe_path { argv[0] }; 
   help_message = make_usage_string_(exe_path.filename().string(), desc, p);
   try {

     store(command_line_parser(argc, argv)
         	    .options(all_options)
         	    .positional(p)
         	    .run(), vm);
     notify(vm);

     if (vm.count("help") ) {
       std::cout << help_message << std::endl;
       exit(amm::ExecutionErrors::NO_ERROR);
     }

     if (vm["version"].as<bool>()) {
       std::cout << SERIAL_BRIDGE_VERSION  << std::endl;
       exit(amm::ExecutionErrors::NO_ERROR);
     }
    
     g_log_level = vm["verbosity"].as<int>();

   } catch (boost::program_options::required_option e) {
     std::cerr << e.what() << "\n\n";
     std::cout << help_message << std::endl;
     exit(amm::ExecutionErrors::ARGUMENT_ERROR);
   } catch (std::exception& e) {
     std::cerr << e.what() << std::endl;
     std::cout << help_message << std::endl;
     exit(amm::ExecutionErrors::ARGUMENT_ERROR);
   }

   LOG_INFO << "Linux Serial_Bridge starting up";
   g_mgr = new AMM::DDSManager<AMMListener>(configFile);
   const int buf_max =  8192;
   char serialport[40];
   char eolchar = '\n';
   int timeout = 500;
   char buf[buf_max];
   std::string serialDevice = vm["device"].as<std::string>();
   strcpy(serialport, serialDevice.c_str());


   g_mgr->InitializeCommand();
   g_mgr->InitializeInstrumentData();
   g_mgr->InitializeSimulationControl();
   g_mgr->InitializePhysiologyModification();
   g_mgr->InitializeRenderModification();
   g_mgr->InitializeAssessment();
   g_mgr->InitializePhysiologyValue();

   g_mgr->InitializeOperationalDescription();
   g_mgr->InitializeModuleConfiguration();
   g_mgr->InitializeStatus();

   g_mgr->CreateOperationalDescriptionPublisher();
   g_mgr->CreateModuleConfigurationPublisher();
   g_mgr->CreateStatusPublisher();

   AMMListener tl;
   
   g_mgr->CreatePhysiologyValueSubscriber(&tl, &AMMListener::onNewPhysiologyValue);
   g_mgr->CreateCommandSubscriber(&tl, &AMMListener::onNewCommand);
   g_mgr->CreateRenderModificationSubscriber(&tl, &AMMListener::onNewRenderModification);
   g_mgr->CreatePhysiologyModificationSubscriber(&tl, &AMMListener::onNewPhysiologyModification);
   g_mgr->CreateSimulationControlSubscriber(&tl, &AMMListener::onNewSimulationControl);

   g_mgr->CreateRenderModificationPublisher();
   g_mgr->CreatePhysiologyModificationPublisher();
   g_mgr->CreateCommandPublisher();
   g_mgr->CreateInstrumentDataPublisher();

   m_uuid.id(g_mgr->GenerateUuidString());

   std::this_thread::sleep_for(std::chrono::milliseconds(250));

   // PublishOperationalDescription();
   // PublishConfiguration();

   std::thread ec(checkForExit);

   fd = serialport_init(serialport, vm["rate"].as<int>());
   if (fd == -1) {
      LOG_ERROR << "Unable to open serial port " << serialport;
      exit(amm::ExecutionErrors::SERIAL_FAILURE);
   }

   LOG_INFO << "Opened port " << serialport;
//    serialport_flush(fd);

   signal(SIGINT, signalHandler);
   signal(SIGTERM, signalHandler);

   LOG_INFO << "Serial_Bridge ready";

   while (!closed) {
      memset(buf, 0, buf_max);  //
      serialport_read_until(fd, buf, eolchar, buf_max, timeout);
      if(g_log_level > 2) {
         LOG_DEBUG << "Read in string: " << buf;
      }
      globalInboundBuffer += buf;
      readHandler();

      while (!transmitQ.empty()) {
         std::string sendStr = transmitQ.front();
         if(g_log_level > 2) {
           LOG_DEBUG << "Writing from transmitQ: " << sendStr;
         }
         rc = serialport_write(fd, sendStr.c_str());
         if (rc == -1) {
            LOG_ERROR << " Error writing to serial port";
         }
         transmitQ.pop();
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
   }

   serialport_close(fd);

   ec.join();

   exit(amm::ExecutionErrors::SUCCESS);
}
