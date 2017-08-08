/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <cassert>
#include <cmath>
#include <ctime>

#include <yarp/os/DummyConnector.h>

#include <iCub/skinDynLib/skinContact.h>

void fillSkinContactWithArbitraryData(iCub::skinDynLib::skinContact & contact)
{
    contact.setActiveTaxels(10);
    contact.setPressure(20.0);
    std::vector<unsigned int> taxelList;
    taxelList.push_back(1);
    taxelList.push_back(2);
    taxelList.push_back(3);
    contact.setTaxelList(taxelList);
    contact.setLinkName("l_upper_forearm");
    contact.setFrameName("l_upper_forearm_dh_frame");
}

void checkSkinContactIsPreserved(iCub::skinDynLib::skinContact & contact,
                          iCub::skinDynLib::skinContact & contactCheck)
{
    double tol = 1e-7;
    yAssert(contact.getActiveTaxels() == contactCheck.getActiveTaxels());
    yAssert(std::fabs(contact.getPressure()-contactCheck.getPressure()) < tol);
    std::vector<unsigned int> taxelList = contact.getTaxelList();
    std::vector<unsigned int> taxelListCheck = contactCheck.getTaxelList();
    yAssert(taxelList.size() == taxelListCheck.size());
    for (int i=0; i < taxelList.size(); i++)
    {
        yAssert(taxelList[i] == taxelListCheck[i]);
    }
    yAssert(contact.getLinkName() == contactCheck.getLinkName());
    yAssert(contact.getFrameName() == contactCheck.getFrameName());
}

void checkAndBenchmarkSkinContactSerialization()
{
    // Create an empty skinContact
    iCub::skinDynLib::skinContact contact, contactCheck;
    fillSkinContactWithArbitraryData(contact);

    // Create a buffer to which write the skin contact
    yarp::os::DummyConnector buffer;

    // Run several times for a reliable benchmark
    double codec_time = 0.0;
    size_t n = 200000;
    clock_t tic = clock();
    contact.write(buffer.getWriter());
    for (size_t i=0; i < n; i++)
    {
        // Write it to a buffer
        contact.write(buffer.getWriter());

        // Read it back
        contactCheck.read(buffer.getReader());

        // Reset the buffer
        buffer.reset();
    }
    clock_t toc = clock();
    codec_time = ((double)(toc - tic) / CLOCKS_PER_SEC)/((double)n);

    std::cerr << "Average time for iCub::skinDynLib::skinContact codec: " << codec_time << std::endl;

    // Check that the data is preserved
    checkSkinContactIsPreserved(contact, contactCheck);
}

void fillDynContactWithArbitraryData(iCub::skinDynLib::dynContact & contact)
{
    contact.setLinkName("l_upper_forearm");
    contact.setFrameName("l_upper_forearm_dh_frame");
}

void checkDynContactIsPreserved(iCub::skinDynLib::dynContact & contact,
                                 iCub::skinDynLib::dynContact & contactCheck)
{
    yAssert(contact.getLinkName() == contactCheck.getLinkName());
    yAssert(contact.getFrameName() == contactCheck.getFrameName());
}

void checkAndBenchmarkDynContactSerialization()
{
    // Create an empty dynContact
    iCub::skinDynLib::dynContact contact, contactCheck;
    fillDynContactWithArbitraryData(contact);

    // Create a buffer to which write the skin contact
    yarp::os::DummyConnector buffer;

    // Run several times for a reliable benchmark
    double codec_time = 0.0;
    size_t n = 200;
    clock_t tic = clock();
    contact.write(buffer.getWriter());
    for (size_t i=0; i < n; i++)
    {
        // Write it to a buffer
        contact.write(buffer.getWriter());

        // Read it back
        contactCheck.read(buffer.getReader());

        // Reset the buffer
        buffer.reset();
    }
    clock_t toc = clock();
    codec_time = ((double)(toc - tic) / CLOCKS_PER_SEC)/((double)n);

    std::cerr << "Average time for iCub::skinDynLib::dynContact codec: " << codec_time << std::endl;

    // Check that the data is preserved
    checkDynContactIsPreserved(contact, contactCheck);
}


int main()
{
    checkAndBenchmarkDynContactSerialization();
    checkAndBenchmarkSkinContactSerialization();
}