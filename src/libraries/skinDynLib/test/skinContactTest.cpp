/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <cassert>
#include <cmath>
#include <ctime>

#include <yarp/os/DummyConnector.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

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
    contact.setForceTorqueEstimateConfidence(5);
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
    yAssert(contact.getForceTorqueEstimateConfidence() == contactCheck.getForceTorqueEstimateConfidence());
}

void checkSkinContactListIsPreserved(iCub::skinDynLib::skinContactList & list,
                                     iCub::skinDynLib::skinContactList & listCheck)
{
    yAssert(list.size() == listCheck.size());
    for (size_t i=0; i < list.size(); i++)
    {
        checkSkinContactIsPreserved(list[i], listCheck[i]);
    }
}

void checkAndBenchmarkSkinContactSerialization()
{
    // Create an empty skinContact
    iCub::skinDynLib::skinContact contact, contactCheck;
    fillSkinContactWithArbitraryData(contact);

    // Create a buffer to which write the skin contact
    yarp::os::DummyConnector buffer;

    // Increase this parameter for a more reliable benchmark
    size_t n = 20;
    double codec_time = 0.0;
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

    // Create a skinContactList to check if it is serialized correctly
    iCub::skinDynLib::skinContactList list, listCheck;
    list.push_back(contact);
    list.push_back(contactCheck);

    buffer.reset();
    list.write(buffer.getWriter());
    listCheck.read(buffer.getReader());

    checkSkinContactListIsPreserved(list, listCheck);

    // Check that the skinContactList can be successfully deserialized as a bottle
    buffer.reset();
    yarp::os::Bottle botCheck;
    list.write(buffer.getWriter());
    bool readBottleOk = botCheck.read(buffer.getReader());
    yAssert(readBottleOk);
    // Check that the bottle has the right size
    std::cerr << "size: " << botCheck.size() << std::endl;
    yAssert(botCheck.size() == list.size());
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

    // Increase this parameter for a more reliable benchmark
    size_t n = 20;
    double codec_time = 0.0;
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

    yarp::os::Network net;
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port;
    iCub::skinDynLib::skinContact contact, contactCheck;
    fillSkinContactWithArbitraryData(contact);
    fillSkinContactWithArbitraryData(contactCheck);
    iCub::skinDynLib::skinContactList list;
    list.push_back(contact);
    list.push_back(contactCheck);
    port.open("/skinDynTest");
    for(int i=0; i < 1000; i++)
    {
        iCub::skinDynLib::skinContactList &skinEvents = port.prepare();
        skinEvents.clear();
        skinEvents.insert(skinEvents.end(), list.begin(), list.end());
        port.write();
        yarp::os::SystemClock::delaySystem(0.1);
    }
    port.close();
}