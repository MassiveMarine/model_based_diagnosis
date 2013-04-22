/*
* Board.cpp provides functionality of a virtual board not real.
*
* Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
* Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
* All rights reserved.
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 

//#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <boost/thread.hpp>
#include <stdlib.h>

int BUFF_SIZE;
int sock, connected, bytes_recieved , True = 1, broadcastFrq;  
char send_data[255] , recv_data[255];
unsigned char nbuffer[255];
unsigned char * p;
int channels;
int MAX_channels = 8;
struct board
{
         char on_off[8];
         float max_vol[8];
         float max_cur[8];
         float pr_vol[8];
         float pr_cur[8];
};
struct board board_info; 
struct sockaddr_in server_addr,client_addr;    
int sin_size;

void set_channels(void)
{
            for(int channel=0;channel<MAX_channels;channel++)
           {
             if((channel%2)==0)
               { 
                 board_info.on_off[channel] = 0;
                 board_info.max_vol[channel] = channel*10+0.1*channel;
                 board_info.max_cur[channel] = channel*20+0.1*channel;
                 board_info.pr_vol[channel] = channel-1+0.1*channel;
                 board_info.pr_cur[channel] = channel+1+0.1*channel;
              }
             else
                {
                 board_info.on_off[channel] = 0;
                 board_info.max_vol[channel] = channel*5+0.1*channel;
                 board_info.max_cur[channel] = channel*8+0.1*channel;
                 board_info.pr_vol[channel] = channel+0.1*channel;
                 board_info.pr_cur[channel] = channel+2+0.1*channel;
                 }
           }        

}

void take_boardSpecifications()
{          
          p = nbuffer;
					channels = MAX_channels;
          char delim = 2;
          char command = 0;
          ushort length = 8*channels+1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;


          char n = (char) channels;
          
         *p=n;
          p++;
          int offset = sizeof(delim)+sizeof(command)+sizeof(length)+sizeof(n);
          
          for(int channel=0;channel<channels;channel++)
          {
          float max_curr = board_info.max_cur[channel];
          float max_vol = board_info.max_vol[channel];
          *((float *)p) = max_curr;
          p+=4;
          *((float *)p) = max_vol;
          p+=4;
          }
          BUFF_SIZE = 4+1+8*channels;
          offset = 5;
          for(int channel=0;channel<channels;channel++)
          {
           offset +=8;
          }
}                

void take_boardMeasurments()
{         
          p = nbuffer;
					channels = 8;
          char delim = 2;
          char command = 2;
          ushort length = 9*channels+1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;


          char n = (char) channels;
          
         *p=n;
          p++;

          for(int channel=0;channel<channels;channel++)
          {
          char on_off = board_info.on_off[channel];
          float pr_curr = board_info.pr_cur[channel];
          float pr_vol = board_info.pr_vol[channel];
          *p=on_off;
          p++;
          *((float *)p) = pr_curr;
          p+=4;
          *((float *)p) = pr_vol;
          p+=4;
          }

          BUFF_SIZE = 4+1+9*channels;
         
					
}

bool check_Command(unsigned char *buf)
{ 
  //printf("\n RECIEVED from Client:\ndelim = %i , command = %i , length = %d" ,*recv_data, *(&recv_data[1]), *(&recv_data[2]),*((float *)(nbuffer+5)));
}
void send_Ack()
{         
          //bool correctCommand=check_Command();
          p = nbuffer;
	  char delim = 2;
          char command = 5;
          ushort length = 1;
          
          *p = delim;
          p++;
          *p = command;
          p++;
          *((ushort *)p) = length;
          p+=2;

          char ecode = 0;
          
         *p=ecode;
          
          BUFF_SIZE = 5;
          send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
          
         
}

void switch_OnOffChannel(char channel, char status)
{
int ch;
int state;
ch = (int) channel;
ch = ch - 48;
state = (int) status;
state = state - 48;
if(state>1)
state = 1;
board_info.on_off[ch] = state;
}

void BoradcastWithFrq(int connected)
{

while(true)
 {
    
    take_boardMeasurments(); 
    send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
    double Frq = ((double)1/broadcastFrq);
    sleep(Frq);
 }
}

void thread_for_client(int connected)
{
            boost::thread broadcaster;

            take_boardSpecifications();
            char command=0;
            while (1)
            { 
              if(command!=4)
              send(connected,(void*)&nbuffer,BUFF_SIZE, 0);
              bytes_recieved = recv(connected,recv_data,255,0);
              
                   
              command = recv_data[1];
              if(command==1)
                   {
                    printf("\n RECIEVED from Client(%d):\ndelim = %i, command = %i,length = %d, BroadCastFrequency = %d", connected ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *((unsigned char *)&recv_data[4]));
		    unsigned char b_frq = *((unsigned char *)&recv_data[4]);
		    broadcastFrq = (int) b_frq;
		    broadcaster.interrupt();
                    broadcaster = boost::thread(BoradcastWithFrq, connected);
                    
                   }
              else if(command==3)
                 { printf("\n RECIEVED from Client(%d):\ndelim = %i, command = %i,length = %i " ,connected ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *(&recv_data[4]), *(&recv_data[5]));
                   send_Ack();
                   take_boardMeasurments();
                 }
              else if(command==4)
                   {
                     switch_OnOffChannel(*(&recv_data[4]),*(&recv_data[5]));
                     printf("\n RECIEVED from Client(%d):\ndelim = %i, command = %i,length = %d,channel = %c, State = %c ", connected ,*recv_data, *(&recv_data[1]), *(&recv_data[2]), *(&recv_data[4]), *(&recv_data[5]));
                    send_Ack();
                   }
              fflush(stdout);
            }
}



int main()
{

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket");
            exit(1);
        }

        if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&True,sizeof(int)) == -1) {
            perror("Setsockopt");
            exit(1);
        }
        
        server_addr.sin_family = AF_INET;         
        server_addr.sin_port = htons(5000);     
        server_addr.sin_addr.s_addr = INADDR_ANY; 
        bzero(&(server_addr.sin_zero),8); 

        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))
                                                                       == -1) {
            perror("Unable to bind");
            exit(1);
        }

        if (listen(sock, 5) == -1) {
            perror("Listen");
            exit(1);
        }
		
	      
        fflush(stdout);
        set_channels();
        boost::thread cthread;
        
        while(1)
        {  

            sin_size = sizeof(struct sockaddr_in);
            printf("TCPServer Waiting for client on port 5000.....\n");
            int client_sock = accept(sock, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size);

            printf(" Got request for connection from (%s , %d)\n",
                   inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
	    cthread = boost::thread(thread_for_client, client_sock);
      }       

      close(sock);
      return 0;
} 

