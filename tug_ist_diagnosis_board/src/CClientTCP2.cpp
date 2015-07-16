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

typedef actionlib::SimpleActionClient<tug_ist_diagnosis_msgs::SystemModelAction> modelClient;
using std::vector;
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
         int no_of_rules;
         int no_of_props;
         int no_of_obs;
         std::string neg_prefix;
         vector<std::string> msg_list;
         
         
         
         
public:

    Diagnosis_Client(ros::NodeHandle nh)
    {      nh_ = nh;
           mdl_sub = nh_.subscribe("/diagnosis_model", 1, &Diagnosis_Client::modelCB, this);
           obs_sub = nh_.subscribe("/observations", 1, &Diagnosis_Client::observationsCB, this);
           ROS_INFO("\nWaiting for the Diagnosis Model Action Server.......");
           modelClient mc("diagnosis_model_server", true);
           tug_ist_diagnosis_msgs::SystemModelGoal goal;
  	   tug_ist_diagnosis_msgs::SystemModelResult result;
  	   goal.goal = 1;
           mc.waitForServer();
           mc.sendGoal(goal);
	   printf("\nGetting model.....");
           mc.waitForResult(ros::Duration(5.0));
  	   if (mc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                printf("\nGot the model!\n");
	   else
                printf("\nCould not get the Model!");
           connect_to_Server();

  }
  
  ~Diagnosis_Client(){
      disconnect_to_Server();
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

  void make_false_rule(tug_ist_diagnosis_msgs::SystemModelResult diag_result) {
           char a[32];
           std::string str="";
           sprintf(a, "%d", no_of_props);
           str.append(a);
           str.append("\r\n\r\n");
           for(int p=0;p<no_of_props;p++){
              str.append(diag_result.props[p].c_str());
              str.append(",");
              str.append(diag_result.neg_prefix.c_str());
              str.append(diag_result.props[p].c_str());
              str.append("->false.\r\n");
           }
           str.append("\r\n\r\n");
           
           char* c=const_cast<char*>(str.data());
	   ROS_INFO("OBS False %s",c);
           send_OBS_to_server(c);
           recieve_from_server(); 
           
  }
  
  void make_SD_rules(tug_ist_diagnosis_msgs::SystemModelResult diag_result){
           char a[32];
           sprintf(a, "%d", no_of_rules);
           std::string str="";
	   str.append(a);
           str.append("\r\n\r\n");
           for(int r=0;r<no_of_rules;r++){
              str.append(diag_result.rules[r].c_str());
              str.append(".\r\n"); 
           }
           str.append("\r\n\r\n");
           
	   char* c=const_cast<char*>(str.data());
           send_SD_to_server(c);
           recieve_from_server();
           
  }
   void observationsCB(const tug_ist_diagnosis_msgs::ObservationsConstPtr & obs_msg){
                ROS_INFO("SZ%d",no_of_obs);
                for(int o=0;o<obs_msg->obs.size();o++){
                   ROS_INFO("%d",o);
                   std::string s = obs_msg->obs[o].c_str();
                   std::string ns = "@";
                   if(s.at(0)=='~'){ 
                       ns = s.substr(1);
                       s = neg_prefix + s.substr(1);
                   }else
                       ns = neg_prefix + s;
                 ROS_INFO("s=%s,ns=%s",s.c_str(),ns.c_str());
                 if (std::find(msg_list.begin(), msg_list.end(), s) != msg_list.end()){
  			// DO NOTHING 
		 } 
                 else {
                       msg_list.push_back(s);
                       std::vector<std::string>::iterator iter=std::find(msg_list.begin(), msg_list.end(), ns);
    		       if(iter != msg_list.end())
        		     msg_list.erase(iter);
        				
                 }
                for (int i=0; i<msg_list.size(); i++)
    			ROS_INFO("---List ITEM = %s,%d,",msg_list[i].c_str(),msg_list.size());							                  
               }// for loop
              
      char a[32];
      std::string str="";
      no_of_obs = msg_list.size();
      sprintf(a, "%d", no_of_obs);
      str.append(a);
      str.append("\r\n\r\n");
      for(int b=0;b<no_of_obs;b++){
           str.append(msg_list[b].c_str());
           str.append(".\r\n");
      }
      str.append("\r\n\r\n");
           
      char* c=const_cast<char*>(str.data());
      ROS_INFO("[%dOBS LIST %s]",no_of_obs,c);
      send_OBS_to_server(c);
      recieve_from_server();
      send_data = "GET sessionA MIN_DIAG\r\nUse-Fault-Modes: true\r\n\r\n\r\n";
      send_QUERY_to_server(send_data);
      recieve_from_server(); 
      ROS_INFO("Observations received....................................%s,%d",obs_msg->obs[0].c_str(),obs_msg->obs.size());
      for(int o=0;o<obs_msg->obs.size();o++){
         ROS_INFO("%s",obs_msg->obs[o].c_str());
      }
  }

  void modelCB(const tug_ist_diagnosis_msgs::SystemModelResultConstPtr & mdl_msg){
           disconnect_to_Server();
           connect_to_Server();
           no_of_rules = mdl_msg->rules.size();
           no_of_props = mdl_msg->props.size();
           neg_prefix = mdl_msg->neg_prefix.c_str();
           
     	   make_SD_rules(*mdl_msg);
           make_false_rule(*mdl_msg);          
           ROS_INFO("model received............................................");
  }
  
  void spin(){
      while( nh_.ok() ){
         ros::spinOnce();
      }
  }

  void publishDiag()
  { ROS_INFO("\n I am a Thread!");
   }

  

};



int main(int argc, char** argv)

{
  ros::init(argc, argv, "Diagnosis_Client_Node");
  ros::NodeHandle n;
  Diagnosis_Client *c = new Diagnosis_Client(n);
  c->spin();
  return 0;
}

