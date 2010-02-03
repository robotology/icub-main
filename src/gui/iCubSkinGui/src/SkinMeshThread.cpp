// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/SkinMeshThread.h>

#include <yarp/os/Time.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;
const int canBufferSize=144;

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
    driver.view(pCanBufferFactory);
    pCanBus->canSetBaudRate(0); //default 1MB/s

    pCanBus->canIdAdd(cardId);
    pCanBus->canIdAdd(cardId+1);
    pCanBus->canIdAdd(cardId+2);
    pCanBus->canIdAdd(cardId+3);
    pCanBus->canIdAdd(cardId+4);
    pCanBus->canIdAdd(cardId+5);

    canBuffer=pCanBufferFactory->createBuffer(canBufferSize);

    printf("... done!\n");

    return true;
}

void SkinMeshThread::run()
{	
    unsigned int canMessages=0;
    bool res=pCanBus->canRead(canBuffer,canBufferSize,&canMessages,true);

    mutex.wait();
    /*
    int triangleId=1+rand()%6;

    unsigned char data[16];
    for (int i=1; i<=12; ++i) data[i]=rand()%256;

    if (rand()%2)
    {
        triangles[triangleId]->setActivationLast5(data); // last 5 bytes
    }
    else
    {
        triangles[triangleId]->setActivationFirst7(data); // first 7 bytes
    }
    */
	for (unsigned int i=0; i<canMessages; i++)
	{
        CanMessage &msg=canBuffer[i];

		if ((msg.getId() & 0xFFFFFFF0) == cardId)
		{
            int triangleId=msg.getId() & 0x0F;

            //printf("%d triangle: ",triangleId);
            //for (int i=0; i<8; ++i) printf("%03d ",msg.getData()[i]);
            //printf("\n");

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