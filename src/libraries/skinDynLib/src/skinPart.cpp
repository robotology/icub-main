#include "iCub/skinDynLib/skinPart.h"

using namespace yarp::math;
using namespace iCub::skinDynLib;

/****************************************************************/
/* SKINPARTBASE WRAPPER
*****************************************************************/
    skinPartBase::skinPartBase() : name("unknown_skin_part"), size(0), version("unknown_version") {}

    skinPartBase::skinPartBase(const skinPartBase &_spb)
    {
        *this = _spb;
    }

    skinPartBase & skinPartBase::operator=(const skinPartBase &_spb)
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        if (this == &_spb)
        {
            return *this;
        }

        name = _spb.name;
        size = _spb.size;
        version = _spb.version;
        return *this;
    }

    void skinPartBase::setName(const std::string &_name)
    {
        name = _name;
    }

    std::string skinPartBase::getName()
    {
        return name;
    }

    void skinPartBase::setSize(int _size)
    {
        size = _size;
    }

    int skinPartBase::getSize()
    {
        return size;
    }

    void skinPartBase::setVersion(const std::string &_version)
    {
        version = _version;
    }

    std::string skinPartBase::getVersion()
    {
        return version;
    }

    
    void skinPartBase::print(int verbosity)
    {
        yDebug("**********\n");
        yDebug("name: %s\t", name.c_str());
        yDebug("total number of taxels: %i\n", size);
        yDebug("version: %s\n", version.c_str());
    }

    std::string skinPartBase::toString(int precision)
    {
        std::stringstream res;
        res << "**********\n" << "Name: " << name << "\tSize: "<< size << "\tVersion: "<< version << std::endl;
        return res.str();
    }

/****************************************************************/
/* SKINPART TAXEL WRAPPER
*****************************************************************/
    skinPart::skinPart()
    {
        spatial_sampling = "taxel";        
    }

    skinPart::skinPart(const std::string &_filePath)
    {
        setTaxelPosesFromFile(_filePath);
    }

    skinPart::skinPart(const skinPart &_sp)
    {
        *this=_sp;
    }

    skinPart & skinPart::operator=(const skinPart &_sp)
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        if (this == &_sp)
        {
            return *this;
        }

        skinPartBase::operator=(_sp);

        spatial_sampling = _sp.spatial_sampling;
        taxel2Repr       = _sp.taxel2Repr;
        repr2TaxelList   = _sp.repr2TaxelList;

        clearTaxels();
        for (std::vector<Taxel*>::const_iterator it = _sp.taxels.begin();
             it != _sp.taxels.end(); ++it)
        {
            taxels.push_back(new Taxel(*(*it)));
        }

        return *this;
    }

    bool skinPart::setTaxelPosesFromFile(const std::string &_filePath, const std::string &_spatial_sampling)
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        // Get the filename from the full absolute path
        std::string filename = "";
        filename = strrchr(_filePath.c_str(), '/');
        filename = filename.c_str() ? filename.c_str() + 1 : _filePath.c_str();

        yarp::os::ResourceFinder rf;
        rf.setDefaultContext("skinGui");            //overridden by --context parameter
        rf.setDefaultConfigFile(_filePath.c_str()); //overridden by --from parameter
        rf.configure(0,NULL);
        
        if (rf.check("name"))
        {
            setName(rf.find("name").asString());
        }
        else
        {
            yWarning("[skinPart::setTaxelPosesFromFile] no name field found. Using filename.");
            // Assign the name and version of the skinPart according to the filename (hardcoded)
            if      (filename == "left_forearm_mesh.txt")    { setName(SkinPart_s[SKIN_LEFT_FOREARM]);    setVersion("V1");   }
            else if (filename == "left_forearm_nomesh.txt")  { setName(SkinPart_s[SKIN_LEFT_FOREARM]);    setVersion("V1");   }
            else if (filename == "left_forearm_V2.txt")      { setName(SkinPart_s[SKIN_LEFT_FOREARM]);    setVersion("V2");   }
            else if (filename == "right_forearm_mesh.txt")   { setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   setVersion("V1");   }
            else if (filename == "right_forearm_nomesh.txt") { setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   setVersion("V1");   }
            else if (filename == "right_forearm_V2.txt")     { setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   setVersion("V2");   }
            else if (filename == "left_hand_V2_1.txt")       { setName(SkinPart_s[SKIN_LEFT_HAND]);       setVersion("V2.1"); }
            else if (filename == "right_hand_V2_1.txt")      { setName(SkinPart_s[SKIN_RIGHT_HAND]);      setVersion("V2.1"); }
            else if (filename == "left_arm_mesh.txt")        { setName(SkinPart_s[SKIN_LEFT_UPPER_ARM]);  setVersion("V1");   }
            else if (filename == "right_arm_mesh.txt")       { setName(SkinPart_s[SKIN_RIGHT_UPPER_ARM]); setVersion("V1");   }
            else if (filename == "torso.txt")                { setName(SkinPart_s[SKIN_FRONT_TORSO]);     setVersion("V1");   }
            else
            {
                yError("[skinPart::setTaxelPosesFromFile] Unexpected skin part file name: %s.\n",filename.c_str());
                return false;
            }
        }
        yTrace("[skinPart] name set to %s",name.c_str());

        if (rf.check("spatial_sampling"))
        {
            std::string _ss=rf.find("spatial_sampling").asString();

            // This lets us override the field without touching the .ini file
            if (_spatial_sampling=="default" && (_ss=="taxel" || _ss=="triangle"))
            {
                spatial_sampling = _ss;
            }
            else if (_spatial_sampling=="taxel" || _spatial_sampling=="triangle")
            {
                spatial_sampling = _spatial_sampling;
            }
            else if ((_spatial_sampling!="default" && _spatial_sampling!="taxel" &&
                      _spatial_sampling!="triangle") && (_ss=="taxel" || _ss=="triangle"))
            {
                spatial_sampling = _ss;   
            }
        }
        else
        {
            yWarning("[skinPart::setTaxelPosesFromFile] no spatial_sampling field found.");
            spatial_sampling = _spatial_sampling;
        }

        yarp::os::Bottle &calibration = rf.findGroup("calibration");
        if (calibration.isNull())
        {
            yWarning("[skinPart::setTaxelPosesFromFile] No calibration group found!");
            yWarning("[skinPart::setTaxelPosesFromFile] Using old convention");
            spatial_sampling = "taxel";
            return setTaxelPosesFromFileOld(_filePath);
        }

        // First item of the bottle is "calibration", so we should not use it
        setSize(calibration.size()-1);
        yarp::sig::Vector taxelPos(3,0.0);
        yarp::sig::Vector taxelNrm(3,0.0);
        yarp::sig::Vector taxelPosNrm(6,0.0);

        for (int i = 1; i < getSize(); i++)
        {
            taxelPosNrm = vectorFromBottle(*(calibration.get(i).asList()),0,6);
            taxelPos = taxelPosNrm.subVector(0,2);
            taxelNrm = taxelPosNrm.subVector(3,5);
            // the NULL taxels will be automatically discarded 
            if (norm(taxelNrm) != 0 || norm(taxelPos) != 0)
            {
                taxels.push_back(new Taxel(taxelPos,taxelNrm,i-1));
            }
        }

        // Let's read the mapping of the taxels onto the center of their triangle
        // even if the spatial_sampling variable is "taxel"
        // (it might come useful later)
        if (rf.check("taxel2Repr"))
        {
            yarp::os::Bottle b = *(rf.find("taxel2Repr").asList());
            
            for (int i = 0; i < getSize(); i++)
            {
                taxel2Repr.push_back(b.get(i).asInt());
            }
            initRepresentativeTaxels();
        }
        else
        {
            yError("[skinPart::setTaxelPosesFromFile] No 'taxel2Repr' field found");
            return false;
        }
           
        return true;
    }

    // see also Compensator::setTaxelPosesFromFile
    // in icub-main/src/modules/skinManager/src/compensator.cpp
    bool skinPart::setTaxelPosesFromFileOld(const std::string &_filePath)
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        std::string       line;
        std::ifstream     posFile;
        yarp::sig::Vector taxelPos(3,0.0);
        yarp::sig::Vector taxelNrm(3,0.0);

        std::string       filename = strrchr(_filePath.c_str(), '/');
        filename = filename.c_str() ? filename.c_str() + 1 : _filePath.c_str();

        // Open File
        posFile.open(_filePath.c_str());  
        if (!posFile.is_open())
        {
            yError("[skinPart::setTaxelPosesFromFileOld] File %s has not been opened!",
                    _filePath.c_str());
            return false;
        }

        // Acquire taxels
        posFile.clear(); 
        posFile.seekg(0, std::ios::beg);//rewind iterator
        for(unsigned int i= 0; getline(posFile,line); i++)
        {
            line.erase(line.find_last_not_of(" \n\r\t")+1);
            if(line.empty())
                    continue;
            std::string number;
            std::istringstream iss(line, std::istringstream::in);
            for(unsigned int j = 0; iss >> number; j++ )
            {
                if(j<3)
                    taxelPos[j]   = strtod(number.c_str(),NULL);
                else
                    taxelNrm[j-3] = strtod(number.c_str(),NULL);
            }

            // the NULL taxels will be automatically discarded
            if (norm(taxelNrm) != 0 || norm(taxelPos) != 0)
            {
                setSize(getSize()+1);
                taxels.push_back(new Taxel(taxelPos,taxelNrm,i));
            }
            else
                setSize(getSize()+1);
        }

        return mapTaxelsOntoThemselves() && initRepresentativeTaxels();
    }

    bool skinPart::mapTaxelsOntoThemselves()
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        for (int i = 0; i < getSize(); ++i)
        {
            bool isIvalidID=false;
            for (int j = 0; j < getTaxelsSize(); ++j)
            {
                if (taxels[j]->getID()==i)
                {
                    isIvalidID=true;
                    break;
                }
            }

            if (isIvalidID)
            {
                taxel2Repr.push_back(int(i));
            }
            else
            {
                taxel2Repr.push_back(-1);
            }
        }

        return true;
    } 

    bool skinPart::initRepresentativeTaxels()
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        std::list<int> mapp(taxel2Repr.begin(), taxel2Repr.end());
        mapp.sort();
        mapp.unique();

        size_t mappsize = mapp.size();
        for (size_t i = 0; i < mappsize; i++)
        {
            repr2TaxelList[mapp.front()]=vectorofIntEqualto(taxel2Repr,mapp.front());
            mapp.pop_front();
        }
        
        return true;
    }

    int skinPart::getTaxelsSize()
    {
         return taxels.size();
    }

    void skinPart::clearTaxels()
    {
        std::lock_guard<std::recursive_mutex> rlg(recursive_mtx);
        while(!taxels.empty())
        {
            if (taxels.back())
            {
                delete taxels.back();
            }
            taxels.pop_back();
        }
        taxels.clear();
    }

    void skinPart::print(int verbosity)
    {
        skinPartBase::print(verbosity);
        yDebug("number of valid taxels: %i", getTaxelsSize());
        yDebug("spatial_sampling:       %s", spatial_sampling.c_str());
        if ((verbosity>=1 && spatial_sampling == "triangle") ||
            (verbosity>=3 && spatial_sampling == "taxel"))
        {
            yDebug("Taxel ID -> Representative ID:");

            for (int i=0; i<size; i++)
            {
                printf("[ %i->%d ]\t",i,taxel2Repr[i]);
            }
            printf("\n");
            
            yDebug("Representative ID -> Taxel IDs:\n");
            for(std::map<int, std::list<unsigned int> >::const_iterator iter_map = repr2TaxelList.begin(); iter_map != repr2TaxelList.end(); ++iter_map)
            {
                std::list<unsigned int> l = iter_map->second;
                printf("\t%d -> {",iter_map->first);
                for(std::list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                {
                    printf("%u, ",*iter_list);
                }
                printf("}\n");
            }    
        }
        if (verbosity>=2)
        {
            yDebug("Taxels:");
            for (size_t i = 0; i < taxels.size(); i++)
                taxels[i]->print(verbosity-3>0?verbosity-3:0);
        }
        yDebug("**********\n");
    }

    std::string skinPart::toString(int precision)
    {
        std::stringstream res(skinPartBase::toString(precision));
        for (size_t i = 0; i < taxels.size(); i++)
            res << taxels[i]->toString(precision);
        res << "**********\n";
        return res.str();
    }

    skinPart::~skinPart()
    {
        clearTaxels();
    }

// empty line to make gcc and Francesco happy
