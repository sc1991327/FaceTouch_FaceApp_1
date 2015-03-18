/* 
    Simple example of sending an OSC message using oscpack.
*/

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024

bool sendMessage(char *msgBegin, char *msgData){

	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
	// example
    /*p << osc::BeginBundleImmediate
		<< osc::BeginMessage( msgBegin ) 
            << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
        << osc::BeginMessage( "/test2" ) 
            << true << 24 << (float)10.8 << "world" << osc::EndMessage
        << osc::EndBundle;*/

	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(msgBegin) << msgData << osc::EndMessage
		<< osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );

	return true;
}

int main(int argc, char* argv[])
{
    (void) argc; // suppress unused parameter warnings
    (void) argv; // suppress unused parameter warnings

	char *msgBegin = "/test1";
	char *msgData = "Hello world";
	sendMessage(msgBegin, msgData);
}

