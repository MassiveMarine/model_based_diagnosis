#include "Client.h"
#include <iostream>"
Client *client;

int main( int argc, char **argv)
{
  client = new Client();
  client->initClient();
  while(1)
   { 
     puts("\n\n****COMMAND MENUE:*****");
     puts("a. Start/Stop Broadcasting\nb. Request Measurments\nc. Channel Switch On/Off");
     puts("Enter Choice:");
     char d[2];
     gets(d);
     if(strcmp(d,"a")==0)
		{ 
		  int frq;
                  puts("Enter Frequency (0=Stop, N=milisec):");
                  std::cin>>frq;
		  puts("got frq");
		  client->start_stopBroadcasting(frq);
		  
		}
          else if(strcmp(d,"b")==0)
               { 
			client->requestMeasurments();
               }
          else if(strcmp(d,"c")==0)
                {
                     char channel[2],status[2];
                     ushort chan;
                     puts("Enter channel#:");
                     gets(channel);
                     puts("Enter Status(0=OFF,1=ON):");
                     gets(status);
                     if(strcmp(status,"0")==0)
                        client->on_offChannel(channel[0],'0');
                     else
                        client->on_offChannel(channel[0],'1');
                     
                     }
     
       
    }
    
}
