#ifndef COMMS_H
#define COMMS_H

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define SERVER_PORT 12345
#define SERVER_IP "141.213.15.85" //AA4.eecs.umich.edu

// Create a struct for the header message
struct MSG_t {
  bool event;
  double intf_rss_dbm;
  double target_intf_rss_dbm;
};

bool send_message(int sock, bool event, double intf_rss_dbm, double target_intf_rss_dbm) {
  MSG_t msg = {
      event,
      intf_rss_dbm,
      target_intf_rss_dbm
      };

  int sent = send(sock, &msg, sizeof(msg), 0);
  if (sent < 0) {
    perror("send");
    close(sock);
    return false;
  }
  return true;
}

int connectToServerSock() {
  int sock;
  struct sockaddr_in server_addr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return -1;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SERVER_PORT);
  server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

  if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    perror("connect");
    close(sock);
    return -1;
  }
  return sock;
}


#endif // COMMS_H