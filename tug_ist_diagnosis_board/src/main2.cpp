#include "Client.h"

Client *client;

int main( int argc, char **argv)
{
  
ros::init(argc, argv,"board_simulator_node");
ros::NodeHandle n;
int port; 
string ip; 
n.param<int>("port", port, 5000);
n.param<string>("ip", ip, "127.0.0.1");
  
  client = new Client(ip,port);
  client->initClient();
  printf("mainClient");
  while(1)
   { 
     puts("\n\n****COMMAND MENUE:*****");
     puts("a. Start/Stop Broadcasting\nb. Request Measurments\nc. Channel Switch On/Off");
     puts("Enter Choice:");
     char d[2];
     gets(d);
     if(strcmp(d,"a")==0)
              client->start_stopBroadcasting();
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
                     puts("Enter Status(0=OFF,1=ON)#:");
                     gets(status);
                     if(strcmp(status,"0")==0)
                        client->on_offChannel(channel[0],'0');
                     else
                        client->on_offChannel(channel[0],'1');
                     
                     }
     
       
    }
    
}
