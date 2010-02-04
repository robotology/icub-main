// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CB__H_
#define _CB__H_

namespace CB {

    /**                                                                                                                                                                          
     * DH Parameter names                                                                                                                                                        
     **/
    enum ParamType
        {
            DH_ALPHA = 0,
            DH_A,
            DH_D,
            DH_THETA,
        };
    
    /**                                                                                                                                                                          
     * the DH Parameter link types                                                                                                                                               
     **/
    enum LinkType
        {
            LINK_TYPE_CONSTANT = 0,
            LINK_TYPE_PRISMATIC,
            LINK_TYPE_REVOLUTE,
            LINK_TYPE_NONINTERFERING,
        };
    
}

#endif
