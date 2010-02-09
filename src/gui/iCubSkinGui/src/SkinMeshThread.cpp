// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/SkinMeshThread.h>

#include <yarp/os/Time.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

bool SkinMeshThread::threadInit()
{
	printf("Image Thread initialising...\n");

	Property prop;
    prop.put("device", "ecan");
    prop.put("CanTxTimeout", 500);
    prop.put("CanRxTimeout", 500);
    prop.put("CanDeviceNum", 0);
    prop.put("CanMyAddress", 0);
    prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

    pCanBus=NULL;
    pCanBufferFactory=NULL;

    driver.open(prop);
    if (!driver.isValid())
    {
        fprintf(stderr, "Error opening PolyDriver check parameters\n");
        return false;
    }

    driver.view(pCanBus);
    
    if (!pCanBus)
    {
        fprintf(stderr, "Error opening /ecan device not available\n");
        return false;
    }

    driver.view(pCanBufferFactory);
    pCanBus->canSetBaudRate(0); //default 1MB/s

    for (int id=0; id<16; ++id)
    {
        if (triangles[id])
        {
            pCanBus->canIdAdd(cardId+id);
        }
    }

    canBuffer=pCanBufferFactory->createBuffer(2*triangleNum);

    printf("... done!\n");

    return true;
}

void SkinMeshThread::run()
{	
    mutex.wait();

    unsigned int canMessages=0;

    bool res=pCanBus->canRead(canBuffer,2*triangleNum,&canMessages,true);

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=canBuffer[i];

        if ((msg.getId() & 0xFFFFFFF0) == cardId)
        {
            int triangleId=msg.getId() & 0x0F;

            if (msg.getData()[0] & 0x80)
            {
                triangles[triangleId]->setActivationLast5(msg.getData()); // last 5 bytes
            }
            else
            {
                triangles[triangleId]->setActivationFirst7(msg.getData()); // first 7 bytes
            }
        }
    }

    mutex.post();
}

void SkinMeshThread::threadRelease()
{
	printf("Skin Mesh Thread releasing...\n");	
	if (pCanBufferFactory) 
    {
        pCanBufferFactory->destroyBuffer(canBuffer);
    }
    driver.close();
    printf("... done.\n");
}