/* Diagnosis Client */
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tug_ist_diagnosis_msgs/SystemModelAction.h>
#include <tug_ist_diagnosis_msgs/Observations.h>

#define POST_SD_HEADER "POST sessionA ADD_SENTENCES SD ATP\r\nNumber-Rules: "
#define POST_OBS_HEADER "POST sessionA ADD_SENTENCES OBS ATP\r\nNumber-Rules: "

class Diagnosis_Client
{

protected:
         ros::NodeHandle nh_;
         ros::Subscriber mdl_sub,obs_sub;
         int sock, bytes_recieved;  
         char recv_data[1024];
         struct hostent *host;
         struct sockaddr_in server_addr;  
         char *send_data;
         pthread_t t;

         
         
         
public:

    Diagnosis_Client(ros::NodeHandle nh)
    {      nh_ = nh;
           connect_to_Server();
           int a =5;
           //pthread_create(&t, 0, &Diagnosis_Client::diag_thread, this);
           //ROS_INFO("Thread return int is =%d",rc);
  }
   ~Diagnosis_Client(){
      disconnect_to_Server();
   }
      
  void test(){
         char *sdata;
         sdata = "4\r\n\r\nNAB(USB),NAB(ARIA)->running(test_node).\r\nNAB(USB),NAB(LASER)->running(test_node1).\r\nAB(USB)->not_running(test_node).\r\nAB(USB)->not_running(test_node1).\r\n\r\n\r\n";
         send_SD_to_server(sdata);
         recieve_from_server();

         sdata = "2\r\n\r\nrunning(test_node), not_running(test_node) -> false.\r\nrunning(test_node1), not_running(test_node1) -> false.\r\n\r\n\r\n";
         send_OBS_to_server(sdata);
         recieve_from_server(); 

         sdata = "2\r\n\r\nnot_running(test_node).\r\nnot_running(test_node1).\r\n\r\n\r\n";
         send_OBS_to_server(sdata);
         recieve_from_server(); 

         sdata = "GET sessionA MIN_DIAG\r\nUse-Fault-Modes: true\r\n\r\n\r\n";
      	 send_QUERY_to_server(sdata);
      	 recieve_from_server(); 
         
         sdata = "CLOSE\r\n\r\n\r\n";
      	 send_QUERY_to_server(sdata);
      	 recieve_from_server(); 



  }
  void set_port(int port) {
  	server_addr.sin_family = AF_INET;     
        server_addr.sin_port = htons(port);   
        server_addr.sin_addr = *((struct in_addr *)host->h_addr);
        bzero(&(server_addr.sin_zero),8); 
  }
  void set_ip(char *ip) {
  	host = gethostbyname(ip);
  }
  void send_SD_to_server(char *data) {
        int size_data = strlen(data);
        char buffer[50+size_data]; 
  	char * ptr;
        strcpy(buffer, POST_SD_HEADER );
  	ptr = strcat( buffer, data );
	ROS_INFO("SD:\n%s",buffer);
  	send(sock,buffer,strlen(buffer), 0);
  }

  void send_OBS_to_server(char *data) {
        int size_data = strlen(data);
        char buffer[50+size_data]; 
  	char * ptr;
        strcpy(buffer,POST_OBS_HEADER );
  	ptr = strcat( buffer, data );
  	send(sock,buffer,strlen(buffer), 0);
        ROS_INFO("\n%s",buffer);
  }


  void send_QUERY_to_server(char *data) {
           
           send(sock,data,strlen(data), 0); 
  }

  void connect_to_Server() {
        set_ip("127.0.0.1");
        set_port(10000);
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("Socket Error!");
            exit(1);
        }

        if (connect(sock, (struct sockaddr *)&server_addr,
                    sizeof(struct sockaddr)) == -1) 
        {
            perror("Connection Error!");
            exit(1);
        }
        ROS_INFO("\nConnected to DiagTCP Server....");
  }
  void disconnect_to_Server() {
        send_QUERY_to_server("CLOSE\r\n\r\n\r\n");
        close(sock);
        ROS_INFO("\nDisconnected from DiagTCP Server....");
  }
  void recieve_from_server() {
       bytes_recieved=recv(sock,recv_data,1024,0);
       recv_data[bytes_recieved] = '\0';
       ROS_INFO("\n%s " , recv_data);
  }

  static void diag_thread(void *threadid){
        ROS_INFO("\n I am a Thread!");
  }

};

int main(int argc, char** argv)

{
  ros::init(argc, argv, "Diagnosis_Client_Node");
  ros::NodeHandle n;
  Diagnosis_Client *c = new Diagnosis_Client(n);
  c->test();
  return 0;
}

