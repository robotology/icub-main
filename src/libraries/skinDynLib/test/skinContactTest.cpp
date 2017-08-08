/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <cassert>
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
}

void checkDataIsPreserved(iCub::skinDynLib::skinContact & contact,
                          iCub::skinDynLib::skinContact & contactCheck)
{
    double tol = 1e-7;
    assert(contact.getActiveTaxels() == contactCheck.getActiveTaxels());
    assert(std::abs(contact.getPressure()-contactCheck.getPressure()) < tol);
    std::vector<unsigned int> taxelList = contact.getTaxelList();
    std::vector<unsigned int> taxelListCheck = contactCheck.getTaxelList();
    assert(taxelList.size() == taxelListCheck.size());
    for (int i=0; i < taxelList.size(); i++)
    {
        assert(taxelList[i] == taxelListCheck[i]);
    }
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
    size_t n = 2000;
    clock_t tic = clock();
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

    // Check that the data is preserved
    checkDataIsPreserved(contact, contactCheck);

    std::cerr << "Average time for iCub::skinDynLib::skinContact codec: " << codec_time << std::endl;
}

int main()
{
    checkAndBenchmarkSkinContactSerialization();
}