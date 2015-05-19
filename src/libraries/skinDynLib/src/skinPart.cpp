#include "iCub/skinDynLib/skinPart.h"

/****************************************************************/
/* SKINPART WRAPPER
*****************************************************************/
    skinPart::skinPart()
    {
        name = SKIN_PART_UNKNOWN;
        size =                 0;
    }

    // skinPart::skinPart(const string _name)
    // {
    //     name = _name;
    //     size =     0;
    // }

    skinPart & skinPart::operator=(const skinPart &spw)
    {
        name           = spw.name;
        size           = spw.size;
        Taxel2Repr     = spw.Taxel2Repr;
        Repr2TaxelList = spw.Repr2TaxelList;
        return *this;
    }

    void skinPart::print(int verbosity)
    {
        yDebug("**********\n");
        yDebug("name: %s\t", SkinPart_s[name].c_str());
        yDebug("size: %i\n", size);
        yDebug("**********\n");
        
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
    }

    string skinPart::toString(int precision)
    {
        stringstream res;
        res << "**********\n" << "Name: " << SkinPart_s[name] << "\tSize: "<< size << endl;
        return res.str();
    }

/****************************************************************/
/* SKINPART TAXEL WRAPPER
*****************************************************************/
    skinPartTaxel & skinPartTaxel::operator=(const skinPartTaxel &spw)
    {
        skinPart::operator=(spw);
        txls     = spw.txls;
        return *this;
    }

    void skinPartTaxel::print(int verbosity)
    {
        skinPart::print(verbosity);
        for (size_t i = 0; i < txls.size(); i++)
            txls[i]->print(verbosity);
        yDebug("**********\n");
    }

    string skinPartTaxel::toString(int precision)
    {
        stringstream res(skinPart::toString(precision));
        for (size_t i = 0; i < txls.size(); i++)
            res << txls[i]->toString(precision);
        res << "**********\n";
        return res.str();
    }

    skinPartTaxel::~skinPartTaxel()
    {
        // while(!txls.empty())
        // {
        //     if (txls.back())
        //     {
        //         delete txls.back();
        //     }
        //     txls.pop_back();
        // }
    }

// empty line to make gcc happy
