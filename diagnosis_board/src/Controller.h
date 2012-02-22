/* Controller Header */
#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <vector>
#include <queue>
#include "Message.h"
#include <actionlib/server/simple_action_server.h>
#include <diagnosis_board/BoardAction.h>
#include <diagnosis_msgs/DBoardMeasurments.h>
#include <diagnosis_msgs/Channel.h>

typedef actionlib::SimpleActionServer<diagnosis_board::BoardAction> boardServer;

using namespace std;
typedef queue<Message> MSGQUEUE;
struct d
     {
      int id;
      float cur;
      float vol;
     };

class Controller
{
public:
   Controller();
   Controller(ros::NodeHandle);
   ~Controller();
   friend void* run_recv_Thread(void*);
   friend void* run_send_Thread(void*);
   void recv_Thread();
   void send_Thread();
   void enable_Sending();
   void initController();
   void CallMessageBroadCasting(char);
   void CallMessageRequest();
   void CallMessageChannelOnOff(char,char);
   void processBuffer(unsigned char *,char);
   void executeAction(const diagnosis_board::BoardGoalConstPtr& goal, boardServer* as);
   bool ControlExit;
      
private:
   void create_threads();
   ros::NodeHandle n_;
	 ros::Publisher pub_board_msr_;
   diagnosis_msgs::DBoardMeasurments board_msr;
   int sock, bytes_recieved;  
   unsigned char send_data[255],recv_data[255];
   unsigned char buffer[255];
   unsigned char * p; 
   struct hostent *host;
   struct sockaddr_in server_addr;
   Message *msg;
   MSGQUEUE rcvQueue;            
   diagnosis_board::BoardFeedback feedback_;
   diagnosis_board::BoardResult result_;
   
};
