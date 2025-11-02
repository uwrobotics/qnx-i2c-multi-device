#include <iostream>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define MY_PULSE_CODE _PULSE_CODE_MINAVAIL

struct my_msg {
    int type;
    char text[64];
};

int main() {
    name_attach_t *attach;
    my_msg msg;
    int rcvid;

    // Attach a name so clients can find us
    attach = name_attach(NULL, "MyServer", 0);
    if (attach == NULL) {
        perror("name_attach");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server started, waiting for messages..." << std::endl;

    while (true) {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            perror("MsgReceive");
            continue;
        }

        if (rcvid == 0) {
            // Handle pulse
            std::cout << "Received a pulse (code " 
                      << msg.type << ")" << std::endl;
        } else {
            // Handle message
            std::cout << "Server received: " << msg.text << std::endl;

            // Reply back to client
            strcpy(msg.text, "Hello from Server!");
            MsgReply(rcvid, 0, &msg, sizeof(msg));
        }
    }

    name_detach(attach, 0);
    return 0;
}

class I2CServer {
    
};