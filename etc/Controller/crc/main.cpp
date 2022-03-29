#include "includes/crc.hpp"

int main(int argc, char *argv[]){
	CentralResiliencyCoordinator::CentralRC CRC;
	
	while(1){
		CRC.communicate();
		usleep(30);  //cycle_time
	}

	return 0;
}
