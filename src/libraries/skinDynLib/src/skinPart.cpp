#include "iCub/skinDynLib/skinPart.h"

/****************************************************************/
/* SKINPARTBASE WRAPPER
*****************************************************************/
    skinPartBase::skinPartBase() : name(SKIN_PART_UNKNOWN), size(0) {}

    skinPartBase::skinPartBase(const SkinPart &_name)
    {
        setName(_name);
        size =     0;
    }

    void skinPartBase::setName(const SkinPart &_name)
    {
        name = _name;
    }

    SkinPart skinPartBase::getName()
    {
        return name;
    }

    int skinPartBase::getSize()
    {
        return size;
    }

    skinPartBase & skinPartBase::operator=(const skinPartBase &spw)
    {
        name           = spw.name;
        size           = spw.size;
        return *this;
    }

    void skinPartBase::print(int verbosity)
    {
        yDebug("**********\n");
        yDebug("name: %s\t", SkinPart_s[name].c_str());
        yDebug("size: %i\n", size);
        yDebug("**********\n");
    }

    string skinPartBase::toString(int precision)
    {
        stringstream res;
        res << "**********\n" << "Name: " << SkinPart_s[name] << "\tSize: "<< size << endl;
        return res.str();
    }

/****************************************************************/
/* SKINPART TAXEL WRAPPER
*****************************************************************/
    skinPartTaxel::skinPartTaxel()
    {
        modality = "full";
    }

    skinPartTaxel::skinPartTaxel(const string &_modality)
    {
        if (_modality == "full" || _modality == "mapping")
        {
            modality = _modality;
        }
        else
        {
            modality = "full";
            yWarning("[skinPartTaxel::skinPartTaxel] modality %s is not valid.",_modality.c_str());
            yWarning("[skinPartTaxel::skinPartTaxel] Using 'full' as default.");
        }
    }

    skinPartTaxel & skinPartTaxel::operator=(const skinPartTaxel &spw)
    {
        skinPartBase::operator=(spw);
        Taxel2Repr     = spw.Taxel2Repr;
        Repr2TaxelList = spw.Repr2TaxelList;
        taxels         = spw.taxels;
        return *this;
    }

    void skinPartTaxel::print(int verbosity)
    {
        skinPartBase::print(verbosity);
        if (verbosity>=4)
        {
            yDebug("\nTaxel ID -> representative ID:\n");

            for (size_t i=0; i<size; i++)
            {
                yDebug("[ %lu -> %d ]\t",i,Taxel2Repr[i]);
                if (i % 8 == 7)
                {
                    yDebug("\n");
                }
            }
            yDebug("\n");
            
            yDebug("Representative ID -> Taxel IDs:\n");
            for(map<unsigned int, list<unsigned int> >::const_iterator iter_map = Repr2TaxelList.begin(); iter_map != Repr2TaxelList.end(); ++iter_map)
            {
                list<unsigned int> l = iter_map->second;
                yDebug("%d -> {",iter_map->first);
                for(list<unsigned int>::const_iterator iter_list = l.begin(); iter_list != l.end(); iter_list++)
                {
                    yDebug("%u, ",*iter_list);
                }
                yDebug("}\n");
            }    
            yDebug("\n");
        }
        yDebug("**********\n");
        for (size_t i = 0; i < taxels.size(); i++)
            taxels[i]->print(verbosity);
        yDebug("**********\n");
    }

    string skinPartTaxel::toString(int precision)
    {
        stringstream res(skinPartBase::toString(precision));
        for (size_t i = 0; i < taxels.size(); i++)
            res << taxels[i]->toString(precision);
        res << "**********\n";
        return res.str();
    }

    bool skinPartTaxel::setTaxelPosesFromFile(const string &filePath, const string &_modality)
    {
        if (_modality == "full" || _modality=="mapping")
        {
            modality = _modality;
        }

        string line;
        ifstream posFile;
        yarp::sig::Vector taxelPos(3,0.0);
        yarp::sig::Vector taxelNorm(3,0.0);

        // Get the filename from the full absolute path
        string filename = strrchr(filePath.c_str(), '/');
        filename = filename.c_str() ? filename.c_str() + 1 : filePath.c_str();

        // Assign the name of the skinPart according to the filename (hardcoded)
        if      (filename == "left_forearm_mesh.txt")    { setName(SKIN_LEFT_FOREARM);  }
        else if (filename == "left_forearm_nomesh.txt")  { setName(SKIN_LEFT_FOREARM);  }
        else if (filename == "right_forearm_mesh.txt")   { setName(SKIN_RIGHT_FOREARM); }
        else if (filename == "right_forearm_nomesh.txt") { setName(SKIN_RIGHT_FOREARM); }
        else if (filename == "left_hand_V2_1.txt")       { setName(SKIN_LEFT_HAND);     }
        else if (filename == "right_hand_V2_1.txt")      { setName(SKIN_RIGHT_HAND);    }
        else
        {
            yError("[skinPart::setTaxelPosesFromFile] Unexpected skin part file name: %s.\n",filename.c_str());
            return false;
        }
           
        // Open File
        posFile.open(filePath.c_str());  
        if (!posFile.is_open())
        {
            yError("[skinPart::setTaxelPosesFromFile] File %s has not been opened.",filePath.c_str());
            return false;
        }

        // Acquire taxels according to the modality used (either "full" or "mapping")
        posFile.clear(); 
        posFile.seekg(0, std::ios::beg);//rewind iterator
        for(unsigned int i= 0; getline(posFile,line); i++)
        {
            // This has been copied from Compensator::setTaxelPosesFromFile()
            // in icub-main/src/modules/skinManager/src/compensator.cpp
            line.erase(line.find_last_not_of(" \n\r\t")+1);
            if(line.empty())
                    continue;
            string number;
            istringstream iss(line, istringstream::in);
            for(unsigned int j = 0; iss >> number; j++ )
            {
                if(j<3)
                    taxelPos[j]    = strtod(number.c_str(),NULL);
                else
                    taxelNorm[j-3] = strtod(number.c_str(),NULL);
            }

            if (modality=="full")
            {
                // the NULL taxels will be automatically discarded (but the size increased)
                if (norm(taxelNorm) != 0 || norm(taxelPos) != 0)
                {
                    size++;
                    taxels.push_back(new Taxel(taxelPos,taxelNorm,i));
                }
                else
                    size++;
            }
            else if (modality=="mapping")
            {
                if (getName() == SKIN_LEFT_FOREARM || getName() == SKIN_RIGHT_FOREARM)
                {
                    // the taxels at the centers of respective triangles [note that i == taxelID == (line in the .txt file +1)]
                    // e.g. first triangle of upper arm is at lines 1-12, center at line 4, thus i=2 
                    if(  (i==3) || (i==15)  ||  (i==27) ||  (i==39) ||  (i==51) ||  (i==63) ||  (i==75) ||  (i==87) ||
                        (i==99) || (i==111) || (i==123) || (i==135) || (i==147) || (i==159) || (i==171) || (i==183) ||
                       (i==207) || (i==255) || (i==291) || (i==303) || (i==315) || (i==339) || (i==351) )
                    {
                        taxels.push_back(new Taxel(taxelPos,taxelNorm,i));
                        size++;
                    }
                    else
                    {
                        size++;
                    }
                }
                else if (getName() == SKIN_LEFT_HAND)
                { //we want to represent the 48 taxels of the palm (ignoring fingertips) with 5 taxels -
                 // manually marking 5 regions of the palm and selecting their "centroids" as the representatives
                    if((i==99) || (i==101) || (i==109) || (i==122) || (i==134)) 
                    {
                        taxels.push_back(new Taxel(taxelPos,taxelNorm,i));
                        size++;
                    }
                    else
                    {
                        size++;
                    }
                }
                else if (getName() == SKIN_RIGHT_HAND)
                { //right hand has different taxel nr.s than left hand 
                    // if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
                    if((i==101) || (i==103) || (i==118) || (i==137)) // || (i==124)) remove one taxel
                    {
                        taxels.push_back(new Taxel(taxelPos,taxelNorm,i));
                        size++;
                    }
                    else
                    {
                        size++;
                    }
                }
            }
            else
            {
                yError("[skinPart::setTaxelPosesFromFile] ERROR: modality is neither 'full' nor 'mapping'!");
                return false;
            }
        }

        if (modality == "full")
        {
            return true;
        }
        else
        {
            return initRepresentativeTaxels();
        }
    }

    bool skinPartTaxel::initRepresentativeTaxels()
    {
        return true;
    }

    int skinPartTaxel::getTaxelSize()
    {
         return taxels.size();
    }

    skinPartTaxel::~skinPartTaxel()
    {
        while(!taxels.empty())
        {
            if (taxels.back())
            {
                delete taxels.back();
            }
            taxels.pop_back();
        }
    }

// empty line to make gcc happy
