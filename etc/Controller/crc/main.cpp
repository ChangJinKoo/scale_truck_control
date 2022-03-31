#include "includes/crc.hpp"

int main(int argc, char *argv[]){
	CentralResiliencyCoordinator::CentralRC CRC;
	
	while(CRC.is_node_running_){
		CRC.communicate();
	}

	return 0;
}
