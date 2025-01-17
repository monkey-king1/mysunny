#include "TCP/TCPServer2.h" 

char TCPServer2::msg[MAXPACKETSIZE];
int TCPServer2::num_client;
int TCPServer2::last_closed;
bool TCPServer2::isonline;
vector<descript_socket2*> TCPServer2::Message;
vector<descript_socket2*> TCPServer2::newsockfd;
std::mutex TCPServer2::mt;

void* TCPServer2::Task(void *arg)
{
	int n;
	struct descript_socket2 *desc = (struct descript_socket2*) arg;
	pthread_detach(pthread_self());

        cerr << "open client[ id:"<< desc->id <<" ip:"<< desc->ip <<" socket:"<< desc->socket<<" send:"<< desc->enable_message_runtime <<" ]" << endl;
	
	while(rclcpp::ok())
	{
		n = recv(desc->socket, msg, MAXPACKETSIZE, 0);
		cerr << "id:      " << msg      << endl;
		if(n != -1) 
		{
			if(n==0)
			{
			   isonline = false;
			   cerr << "close client[ id:"<< desc->id <<" ip:"<< desc->ip <<" socket:"<< desc->socket<<" ]" << endl;
			   last_closed = desc->id;
			   close(desc->socket);

			   int id = desc->id;
			   auto new_end = std::remove_if(newsockfd.begin(), newsockfd.end(),
                				           		   [id](descript_socket2 *device)
		                              				   { return device->id == id; });
			   newsockfd.erase(new_end, newsockfd.end());

			   if(num_client>0) num_client--;
			   break;
			}
			else if(n>=1)
			{
				static int total_rcvnum=0;
				static std::string s_rcvmsg;
				if(msg[n-1]!='\0')
				{
					msg[n]='\0';
					total_rcvnum=total_rcvnum+n;
					std::string rcvmsg=(char*)msg;
					s_rcvmsg=s_rcvmsg+rcvmsg;

					cerr << "id:      " << s_rcvmsg.c_str()      << endl;
				}
				else
				{
					msg[n]='\0';
					total_rcvnum=total_rcvnum+n;
					std::string rcvmsg=(char*)msg;
					s_rcvmsg=s_rcvmsg+rcvmsg;

					cerr << "id:      " << s_rcvmsg.c_str()      << endl;

					desc->message.resize(s_rcvmsg.size());
					for(int i=0;i<n;i++)
					{
						desc->message[i]=s_rcvmsg[i];
					}
					s_rcvmsg.clear();
					total_rcvnum=0;

					std::lock_guard<std::mutex> guard(mt);
					Message.push_back( desc );

					
				}
			}
			
		}
	//	usleep(600);
		sleep(0);
    }
	if(desc != NULL)
	{
		delete desc;
		desc = NULL;
	}
	cerr << "exit thread: " << this_thread::get_id() << endl;
	pthread_exit(NULL);
	
	return 0;
}

int TCPServer2::setup(int port, vector<int> opts)
{
	int opt = 1;
	isonline = false;
	last_closed = -1;
	sockfd = socket(AF_INET,SOCK_STREAM,0);
 	memset(&serverAddress,0,sizeof(serverAddress));

	for(unsigned int i = 0; i < opts.size(); i++) {
		if( (setsockopt(sockfd, SOL_SOCKET, opts.size(), (char *)&opt, sizeof(opt))) < 0 ) {
			cerr << "Errore setsockopt" << endl; 
      			return -1;
	      	}
	}

	serverAddress.sin_family      = AF_INET;
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port        = htons(port);

	if((::bind(sockfd,(struct sockaddr *)&serverAddress, sizeof(serverAddress))) < 0){
		cerr << "Errore bind" << endl;
		return -1;
	}
	
 	if(listen(sockfd,5) < 0){
		cerr << "Errore listen" << endl;
		return -1;
	}
	num_client = 0;
	isonline = true;
	return 0;
}

void TCPServer2::accepted()
{
	socklen_t sosize    = sizeof(clientAddress);
	descript_socket2 *so = new descript_socket2;
	so->socket          = accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
	so->id              = num_client;
	so->ip              = inet_ntoa(clientAddress.sin_addr);
	newsockfd.push_back( so );
	cerr << "accept client[ id:" << newsockfd[num_client]->id << 
	                      " ip:" << newsockfd[num_client]->ip << 
		              " handle:" << newsockfd[num_client]->socket << " ]" << endl;
	pthread_create(&serverThread[num_client], NULL, &Task, (void *)newsockfd[num_client]);
	isonline=true;
	num_client++;
}

vector<descript_socket2*> TCPServer2::getMessage()
{
	std::lock_guard<std::mutex> guard(mt);
	return Message;
}

void TCPServer2::Send(string msg, int id)
{
	if(send(newsockfd[id]->socket,msg.c_str(),msg.length(),MSG_NOSIGNAL)==-1)
	{
		clean(newsockfd[id]->socket);
		close(newsockfd[id]->socket);
	}
}

void TCPServer2::Send(char *msg, int length, int id)
{
	if(send(newsockfd[id]->socket,msg,length,MSG_NOSIGNAL)==-1)
	{
		clean(newsockfd[id]->socket);
		close(newsockfd[id]->socket);
	}
}

int TCPServer2::get_last_closed_sockets()
{
	return last_closed;
}

void TCPServer2::clean(int id)
{
	Message[id] = NULL;
	memset(msg, 0, MAXPACKETSIZE);
}

string TCPServer2::get_ip_addr(int id)
{
	return newsockfd[id]->ip;
}

bool TCPServer2::is_online() 
{
	return isonline;
}

void TCPServer2::detach(int id)
{
	close(newsockfd[id]->socket);
	newsockfd[id]->ip = "";
	newsockfd[id]->id = -1;
	newsockfd[id]->message.clear();
} 

void TCPServer2::closed() 
{
	close(sockfd);
}

