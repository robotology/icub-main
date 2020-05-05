#include "iCub/skinDynLib/iCubSkin.h"

using namespace iCub::skinDynLib;

/****************************************************************/
/* ICUBSKIN WRAPPER
*****************************************************************/
    iCubSkin::iCubSkin()
    {
        configureSkinFromFile();
    }

    iCubSkin::iCubSkin(const std::string &_from)
    {
        configureSkinFromFile(_from);
    }

    iCubSkin::iCubSkin(const std::string &_from, const std::string &_context)
    {
        configureSkinFromFile(_from,_context);
    }

    bool iCubSkin::configureSkinFromFile(const std::string &_from,
                                         const std::string &_context)
    {
        skin.clear();

        yarp::os::ResourceFinder skinRF;
        skinRF.setDefaultContext(_context.c_str());     //overridden by --context parameter
        skinRF.setDefaultConfigFile(_from.c_str());     //overridden by --from parameter
        skinRF.configure(0,NULL);

        yarp::os::Bottle &skinConf = skinRF.findGroup("SKIN_EVENTS");

        if(!skinConf.isNull())
        {
            yInfo("[iCubSkin] SKIN_EVENTS section found");

            yarp::os::Bottle* skinPartList;
            if(skinConf.check("skinParts"))
            {
                skinPartList = skinConf.find("skinParts").asList();
            }
            else
            {
                yError("[iCubSkin] No skinParts field found!");
                return false;
            }

            yarp::os::Bottle *taxelPosFiles;
            if(skinConf.check("taxelPositionFiles"))
            {
                taxelPosFiles = skinConf.find("taxelPositionFiles").asList();
            }
            else
            {
                yError("[iCubSkin] No taxelPositionFiles field found!");
                return false;
            }

            for (int i = 0; i < skinPartList->size(); ++i)
            {
                std::string taxelPosFile = taxelPosFiles->get(i).asString().c_str();
                std::string filePath     = skinRF.findFile(taxelPosFile.c_str());
                if (filePath!="")
                {
                    yInfo("[iCubSkin] filePath [%i] %s\n",i,filePath.c_str());
                    skinPart p;
                    if (p.setTaxelPosesFromFile(filePath))
                    {
                        skin.push_back(p);
                    }
                }
            }
        }
        else
        {
            yError("[iCubSkin] No skin configuration files found.");
            return false;
        }

        if (skin.size()==0)
        {
            yError("[iCubSkin] No valid ini file found. Skin is empty!");
            return false;
        }

        return true;
    }

void iCubSkin::print(int verbosity)
{
    yDebug("********************\n");
    yDebug("iCubSkin size %zi",skin.size());
    for (size_t i = 0; i < skin.size(); ++i)
    {
        skin[i].print(verbosity);
    }
    yDebug("********************\n");
}

// empty line to make gcc happy
