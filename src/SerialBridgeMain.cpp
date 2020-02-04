#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <queue>
#include <stack>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <iostream>

#include "amm_std.h"

extern "C" {
#include "Serial/arduino-serial-lib.h"
}

#include "tinyxml2.h"

#define PORT_LINUX "/dev/tty96B0"
#define BAUD 115200

using namespace AMM;
using namespace std;
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

const string capabilityPrefix = "CAPABILITY=";
const string settingsPrefix = "SETTINGS=";
const string statusPrefix = "STATUS=";
const string configPrefix = "CONFIG=";
const string modulePrefix = "MODULE_NAME=";
const string registerPrefix = "REGISTER=";
const string keepHistoryPrefix = "KEEP_HISTORY=";
const string keepAlivePrefix = "[KEEPALIVE]";
const string loadScenarioPrefix = "LOAD_SCENARIO:";
const string haltingString = "HALTING_ERROR";
const string sysPrefix = "[SYS]";
const string actPrefix = "[ACT]";
const string loadPrefix = "LOAD_STATE:";

std::vector<std::string> subscribedTopics;
std::vector<std::string> publishedTopics;
std::map<std::string, std::string> subMaps;
std::map<std::string, std::map<std::string, std::string>> equipmentSettings;

std::queue<std::string> transmitQ;

int fd = -1;
int rc;


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
          map<string, string>::iterator i = subMaps.find(hfname);
          if (i == subMaps.end()) {
             messageOut << "[AMM_Node_Data]" << n.name() << "=" << n.value() << std::endl;
          } else {
             messageOut << "[" << i->first << "]" << n.value() << std::endl;
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
          map<string, string>::iterator i = subMaps.find(n.name());
          if (i == subMaps.end()) {
             messageOut << "[AMM_Node_Data]" << n.name() << "=" << n.value() << std::endl;
          } else {
             messageOut << "[" << i->first << "]" << n.value() << std::endl;
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
AMM::DDSManager<AMMListener> *mgr = new AMM::DDSManager<AMMListener>(configFile);
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
   mgr->WriteInstrumentData(i);
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
         mgr->WriteCommand(cmdInstance);
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
               mgr->WriteOperationalDescription(od);

               initializing = false;
            }

            subscribedTopics.clear();
            publishedTopics.clear();
            tinyxml2::XMLNode *caps = mod->FirstChildElement("capabilities");

            if (caps) {
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
                     for (tinyxml2::XMLNode *sub = subs->FirstChildElement(
                        "topic"); sub; sub = sub->NextSibling()) {
                        tinyxml2::XMLElement *s = sub->ToElement();
                        std::string subTopicName = s->Attribute("name");


                        if (s->Attribute("name")) {
                           std::string subname = s->Attribute("name");
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
            mgr->WriteRenderModification(renderMod);
         } else if (topic == "AMM_Physiology_Modification") {
            AMM::PhysiologyModification physMod;
            physMod.type(modType);
            physMod.data(modPayload);
            //physMod.location().description(modLocation);
            mgr->WritePhysiologyModification(physMod);
         } else if (topic == "AMM_Performance_Assessment") {
            AMM::Assessment assessment;
            assessment.comment(modInfo);
            mgr->WriteAssessment(assessment);
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
   mgr->WriteOperationalDescription(od);
}

void PublishConfiguration() {
   AMM::ModuleConfiguration mc;
   auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
   mc.timestamp(ms);
   mc.module_id(m_uuid);
   mc.name(moduleName);
   const std::string configuration = AMM::Utility::read_file_to_string("config/serial_bridge_configuration.xml");
   mc.capabilities_configuration(configuration);
   mgr->WriteModuleConfiguration(mc);
}


static void show_usage(const std::string &name) {
   std::cerr << "Usage: " << name << " <option(s)>"
             << "\nOptions:\n" << std::endl
             << "\t-p Linux COM port (defaults to " << PORT_LINUX << ")" << std::endl
             << "\t-b COM port baud rate (defaults to " << BAUD << ")" << std::endl
             << "\t-h,--help\t\tShow this help message\n"
             << std::endl;
}

int main(int argc, char *argv[]) {
   static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
   plog::init(plog::verbose, &consoleAppender);

   LOG_INFO << "Linux Serial_Bridge starting up";
   std::string sPort = PORT_LINUX;
   int baudRate = BAUD;


   for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if ((arg == "-h") || (arg == "--help")) {
         show_usage(argv[0]);
         return 0;
      }

      if (arg == "-b") {
         if (i + 1 < argc) {
            baudRate = stoi(argv[i++]);
         } else {
            LOG_ERROR << arg << " option requires one argument.";
            return 1;
         }
      }

      if (arg == "-p") {
         if (i + 1 < argc) {
            sPort = argv[i++];
         } else {
            LOG_ERROR << arg << " option requires one argument.";
            return 1;
         }
      }
   }

   const int buf_max = 8192;
   char serialport[40];
   char eolchar = '\n';
   int timeout = 500;
   char buf[buf_max];
   strcpy(serialport, sPort.c_str());


   mgr->InitializeCommand();
   mgr->InitializeInstrumentData();
   mgr->InitializeSimulationControl();
   mgr->InitializePhysiologyModification();
   mgr->InitializeRenderModification();
   mgr->InitializeAssessment();
   mgr->InitializePhysiologyValue();

   mgr->InitializeOperationalDescription();
   mgr->InitializeModuleConfiguration();
   mgr->InitializeStatus();

   mgr->CreateOperationalDescriptionPublisher();
   mgr->CreateModuleConfigurationPublisher();
   mgr->CreateStatusPublisher();

   AMMListener tl;
   
   mgr->CreatePhysiologyValueSubscriber(&tl, &AMMListener::onNewPhysiologyValue);
   mgr->CreateCommandSubscriber(&tl, &AMMListener::onNewCommand);
   mgr->CreateRenderModificationSubscriber(&tl, &AMMListener::onNewRenderModification);
   mgr->CreatePhysiologyModificationSubscriber(&tl, &AMMListener::onNewPhysiologyModification);

   mgr->CreateRenderModificationPublisher();
   mgr->CreatePhysiologyModificationPublisher();
   mgr->CreateCommandPublisher();
   mgr->CreateInstrumentDataPublisher();

   m_uuid.id(mgr->GenerateUuidString());

   std::this_thread::sleep_for(std::chrono::milliseconds(250));

   // PublishOperationalDescription();
   // PublishConfiguration();

   std::thread ec(checkForExit);

   fd = serialport_init(serialport, baudRate);
   if (fd == -1) {
      LOG_ERROR << "Unable to open serial port " << serialport;
      exit(EXIT_FAILURE);
   }

   LOG_INFO << "Opened port " << serialport;
//    serialport_flush(fd);

   signal(SIGINT, signalHandler);
   signal(SIGTERM, signalHandler);

   LOG_INFO << "Serial_Bridge ready";

   while (!closed) {
      memset(buf, 0, buf_max);  //
      serialport_read_until(fd, buf, eolchar, buf_max, timeout);
      //        LOG_DEBUG << "Read in string: " << buf;
      globalInboundBuffer += buf;
      readHandler();

      while (!transmitQ.empty()) {
         std::string sendStr = transmitQ.front();
         // LOG_DEBUG << "Writing from transmitQ: " << sendStr;
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

   exit(EXIT_SUCCESS);
}
