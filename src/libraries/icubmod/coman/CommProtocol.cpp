/*
	CommProtocol.cpp

	Copyright (C) 2010 Italian Institute of Technology

	Developer:
        Sabino Colonna (2010-, sabino.colonna@iit.it)
    
*/

#include <string.h>
#include <CommProtocol.hpp>

CommPacket::CommPacket(int cmdId) :
	cmdType(cmdsInfo[cmdId].cmdType),
	cmdId(cmdsInfo[cmdId].cmdId),
	payloadSize(0),
	readOffset(HEADER_SIZE),
	writeOffset(HEADER_SIZE)
{
    memset((void*)content, 0, HEADER_SIZE+MAX_PAYLOAD_SIZE+1);
}

int CommPacket::appendData(const char *data, int nBytes)
{
	if (writeOffset - HEADER_SIZE + nBytes <= MAX_PAYLOAD_SIZE)
	{
		for (int i = 0; i < nBytes; i++) content[writeOffset++] = data[i];
		payloadSize += nBytes; // Update payload size
	}
	else return -1;

	return 0;
}

char CommPacket::getPayloadSize()
{
	return content[(int)OFFSET_PAYLOAD_SIZE];
}

int CommPacket::readData(char *data, int nBytes)
{
	if (readOffset - HEADER_SIZE + nBytes <= payloadSize)
		for (int i = 0; i < nBytes; i++) data[i] = content[readOffset++];
	else return -1;

	return 0;
}

//////////////////////////////////////////////////////////////
//                     Private methods                      //
//////////////////////////////////////////////////////////////

bool CommPacket::verifyHeader()
{
	return (content[(int)OFFSET_CMD_TYPE] == cmdType &&	content[(int)OFFSET_CMD_ID] == cmdId);
}

void CommPacket::fillHeader()
{
	content[(int)OFFSET_CMD_TYPE] = cmdType;
	content[(int)OFFSET_PAYLOAD_SIZE] = payloadSize;
	content[(int)OFFSET_CMD_ID] = cmdId;
}

bool CommPacket::verifyChecksum()
{
	int actualSize = HEADER_SIZE + payloadSize;
	char readChecksum = content[actualSize];
	char computedChecksum = 0;

	for (int i = 0; i < actualSize; i++) computedChecksum -= content[i];
	return (computedChecksum == readChecksum);
}

void CommPacket::setChecksum()
{
	char checksum = 0;
	int actualSize = HEADER_SIZE + payloadSize;

	for (int i = 0; i < actualSize; i++) checksum -= content[i];
	content[actualSize] = checksum;
}

//////////////////////////////////////////////////////////////
//                      TCPCommPacket                       //
//////////////////////////////////////////////////////////////

TCPCommPacket::TCPCommPacket(int cmdId) :
	CommPacket(cmdId)
{
}

int TCPCommPacket::sendToTCPSocket(int socketId)
{
	fillHeader();
	setChecksum();

	int bytesToSend = HEADER_SIZE + content[(int)OFFSET_PAYLOAD_SIZE] + 1; // 1 = checksum
	if (send(socketId, content, bytesToSend, 0) != bytesToSend) {
        return -1;
    }

	return 0;
}

int TCPCommPacket::recvFromTCPSocket(int socketId)
{
	int retValue = -1;

	// Receive header
	if ((retValue = recv(socketId, content, HEADER_SIZE, 0)) == HEADER_SIZE)
	{
		if (verifyHeader())
		{
			payloadSize = content[(int)OFFSET_PAYLOAD_SIZE];
			// Receive payload
			if ((retValue = recv(socketId, &content[readOffset], payloadSize+1, 0)) == payloadSize+1) // 1 = checksum
				if (verifyChecksum()) retValue = 0;
		}
	}

	return retValue;
}

//////////////////////////////////////////////////////////////
//                      UDPCommPacket                       //
//////////////////////////////////////////////////////////////

UDPCommPacket::UDPCommPacket(int cmdId) :
	CommPacket(cmdId)
{
}

int UDPCommPacket::sendToUDPSocket(int socketId, sockaddr *to, socklen_t toLen)
{
	fillHeader();
	setChecksum();

	int bytesToSend = HEADER_SIZE + content[(int)OFFSET_PAYLOAD_SIZE] + 1; // 1 = checksum
	if (sendto(socketId, content, bytesToSend, 0, to, toLen) != bytesToSend)
		return -1;

	return 0;
}

int UDPCommPacket::recvFromUDPSocket(int socketId, sockaddr *from, socklen_t *fromLen)
{
	int retValue = 0;

	if ((retValue = recvfrom(socketId, content, HEADER_SIZE+MAX_PAYLOAD_SIZE+1, 0, from, fromLen)) > 0)
	{
		payloadSize = content[(int)OFFSET_PAYLOAD_SIZE];
		if (!verifyHeader() || !verifyChecksum()) retValue = -1;
	}

	return retValue;
}
