#include "iCub/skinDynLib/skinPart.h"

using namespace yarp::math;
using namespace iCub::skinDynLib;

/****************************************************************/
/* SKINPARTBASE WRAPPER
*****************************************************************/
    skinPartBase::skinPartBase() : name("unknown_skin_part"), size(0) {}

    skinPartBase::skinPartBase(const skinPartBase &_spb)
    {
        *this = _spb;
    }

    skinPartBase & skinPartBase::operator=(const skinPartBase &_spb)
    {
        if (this == &_spb)
        {
            return *this;
        }

        name = _spb.name;
        size = _spb.size;
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

    void skinPartBase::print(int verbosity)
    {
        yDebug("**********\n");
        yDebug("name: %s\t", name.c_str());
        yDebug("size: %i\n", size);
    }

    std::string skinPartBase::toString(int precision)
    {
        std::stringstream res;
        res << "**********\n" << "Name: " << name << "\tSize: "<< size << std::endl;
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
        if (this == &_sp)
        {
            return *this;
        }

        skinPartBase::operator=(_sp);
        Taxel2Repr     = _sp.Taxel2Repr;
        Repr2TaxelList = _sp.Repr2TaxelList;
        clearTaxels();
        for (std::vector<Taxel*>::const_iterator it = _sp.taxels.begin();
             it != _sp.taxels.end(); ++it)
        {
            taxels.push_back(new Taxel(*(*it)));
        }

        return *this;
    }

    void skinPart::print(int verbosity)
    {
        skinPartBase::print(verbosity);
        yDebug("spatial_sampling: %s", spatial_sampling.c_str());
        yDebug("");
        if (verbosity>=1 && spatial_sampling == "patch")
        {
            yDebug("Taxel ID -> Representative ID:");

            for (size_t i=0; i<size; i++)
            {
                printf("[ %lu -> %d ]\t",i,Taxel2Repr[i]);
            }
            printf("\n");
            
            yDebug("Representative ID -> Taxel IDs:\n");
            for(std::map<int, std::list<unsigned int> >::const_iterator iter_map = Repr2TaxelList.begin(); iter_map != Repr2TaxelList.end(); ++iter_map)
            {
                std::list<unsigned int> l = iter_map->second;
                printf("\t%d -> {",iter_map->first);
                for(std::list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                {
                    printf("%u, ",*iter_list);
                }
                printf("}\n");
            }    
            yDebug("\n");
        }
        if (verbosity>=2)
        {
            for (size_t i = 0; i < taxels.size(); i++)
                taxels[i]->print(verbosity-2);
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

    bool skinPart::setTaxelPosesFromFile(const std::string &_filePath, const std::string &_spatial_sampling)
    {
        // Get the filename from the full absolute path
        std::string filename = "";
        filename = strrchr(_filePath.c_str(), '/');
        filename = filename.c_str() ? filename.c_str() + 1 : _filePath.c_str();

        yarp::os::ResourceFinder rf;
        rf.setVerbose(false);
        rf.setDefaultContext("skinGui");           //overridden by --context parameter
        rf.setDefaultConfigFile(_filePath.c_str()); //overridden by --from parameter
        rf.configure(0,NULL);
        rf.setVerbose(true);
        
        if (rf.check("name"))
        {
            setName(rf.find("name").asString());
        }
        else
        {
            yWarning("[skinPart:setTaxelPosesFromFile] no name field found. Using filename.");
            // Assign the name of the skinPart according to the filename (hardcoded)
            if      (filename == "left_forearm_mesh.txt")    { setName("skin_left_forearm");  }
            else if (filename == "left_forearm_nomesh.txt")  { setName("skin_left_forearm");  }
            else if (filename == "right_forearm_mesh.txt")   { setName("skin_right_forearm"); }
            else if (filename == "right_forearm_nomesh.txt") { setName("skin_right_forearm"); }
            else if (filename == "left_hand_V2_1.txt")       { setName("skin_left_hand");     }
            else if (filename == "right_hand_V2_1.txt")      { setName("skin_right_hand");    }
            else
            {
                yError("[skinPart::setTaxelPosesFromFile] Unexpected skin part file name: %s.\n",filename.c_str());
                return false;
            }
        }
        yTrace("[skinPart] name set to %s",name.c_str());

        if (rf.check("spatial_sampling"))
        {
            spatial_sampling = rf.find("spatial_sampling").asString();
        }
        else
        {
            yWarning("[skinPart::setTaxelPosesFromFile] no spatial_sampling field found.");
        }

        yarp::os::Bottle &calibration = rf.findGroup("calibration");
        if (calibration.isNull())
        {
            yError("[skinPart::setTaxelPosesFromFile] No calibration group found!");
            return false;
        }

        // First item of the bottle is "calibration", so we should not use it
        setSize(calibration.size()-1);
        yarp::sig::Vector taxelPos(3,0.0);
        yarp::sig::Vector taxelNrm(3,0.0);
        yarp::sig::Vector taxelPosNrm(6,0.0);

        for (size_t i = 1; i < getSize(); i++)
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

        if (spatial_sampling=="patch")
        {
            if (rf.check("taxel2Patch"))
            {
                yarp::os::Bottle b = *(rf.find("taxel2Patch").asList());
                
                for (size_t i = 0; i < getSize(); i++)
                {
                    Taxel2Repr.push_back(b.get(i).asInt());
                }
                initRepresentativeTaxels();
            }
            else
            {
                yError("[skinPart::setTaxelPosesFromFile] No taxel2Patch field found");
                return false;
            }
        }
           
        return true;
    }

    bool skinPart::initRepresentativeTaxels()
    {
        if (spatial_sampling != "patch")
        {
            yError("[skinPart::initRepresentativeTaxels] spatial_sampling is not 'patch'");
            return false;
        }

        std::list<int> mapp(Taxel2Repr.begin(), Taxel2Repr.end());
        mapp.sort();
        mapp.unique();

        for (size_t i = 0; i < mapp.size(); i++)
        {
            Repr2TaxelList[mapp.front()]=vectorofIntEqualto(Taxel2Repr,mapp.front());
            mapp.pop_front();
        }
        
        return true;
    }

    int skinPart::getTaxelSize()
    {
         return taxels.size();
    }

    void skinPart::clearTaxels()
    {
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

    skinPart::~skinPart()
    {
        clearTaxels();
    }

// empty line to make gcc and Francesco happy
