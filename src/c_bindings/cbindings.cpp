
#include "cbindings.h"

namespace CBindings {

int SUCCESS = 1;
int FAIL = 0;
char CBINDINGS_ERROR_MESSAGE[4096];

void set_error_message(std::exception &ex) {
    std::string msg = ex.what();
    msg.copy(CBINDINGS_ERROR_MESSAGE, msg.length(), 0);
    CBINDINGS_ERROR_MESSAGE[msg.length()] = '\0';
}

char* get_error_message() {
	return CBINDINGS_ERROR_MESSAGE;
}

}