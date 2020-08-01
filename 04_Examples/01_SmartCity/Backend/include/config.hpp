#ifndef __CONFIG_HPP 
#define __CONFIG_HPP
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

#define CONF(t,n) Config::get()->get<t>(n)
#define CONF_OBJS(n) Config::get()->getObjects(n)

#define CONF_BOT(i, n) Config::get()->getBotProperty<std::string>(i, n)

/**
*   Singleton Config Class.
*/
class Config {
public:
    typedef std::vector<std::unordered_map<std::string,std::string>> ListOfObjects;

    /**
    *  Function that returns the property value
    */
    template<typename T> T get(const std::string& name) const {
        try {
            return this->m_sConfig[name].as<T>();
        } catch( const YAML::InvalidNode& ex ) {
            throw std::runtime_error( std::string( "Property not found [" ) + name + "] " + ex.what() );
        } // catch
    }
   
    template<typename T> T getBotProperty(int botId, const std::string& name) const {
        try {
            // TODO: handle exception when yaml file is not found
            YAML::Node doc = YAML::LoadFile("bot_config/bot_" + std::to_string(botId) + ".yaml" );
            return doc[name].as<T>();
        } catch( const YAML::InvalidNode& ex ) {
            throw std::runtime_error( std::string( "Property not found [" ) + name + "] " + ex.what() );
        } // catch
    }

    ListOfObjects getObjects(const std::string& name) const {
        try {
            ListOfObjects objs;
            YAML::Node doc = this->m_sConfig[name];
            for ( unsigned i = 0; i < doc.size(); ++i ) {
                std::unordered_map<std::string, std::string> attributes;
                for(YAML::const_iterator it=doc[i].begin();it!=doc[i].end();++it) {
                    std::string key = it->first.as<std::string>(), value = it->second.as<std::string>();
                    attributes.insert( std::make_pair( key, value ) );
                } // for
                objs.push_back( attributes );
            } // for
            return objs;
        } catch( const YAML::InvalidNode& ex ) {
            throw std::runtime_error( std::string( "Property not found [" ) + name + "] " + ex.what() );
        } // catch
    }

    /**
    *   Funtion to create the instance of config class.
    *   @return singleton object of Cconfig class..
    */
    static Config* get() {
        if (m_pThis == NULL){
            m_pThis = new Config();
        }
        return m_pThis;
    }
private:
    /**
    *    Default constructor for the Config class.
    */
    inline Config() {
        this->m_sConfig = YAML::LoadFile("config.yaml");
    }
    /**
    *   copy constructor for the Config class.
    */
    Config(const Config&){};             // copy constructor is private
    /**
    *   assignment operator for the Config class.
    */
    Config& operator=(const Config&){ return *this; };  // assignment operator is private

    YAML::Node m_sConfig;

    /**
    *   Singleton config class object pointer.
    **/
    inline static Config* m_pThis = NULL;
};

#endif // __CONFIG_HPP
