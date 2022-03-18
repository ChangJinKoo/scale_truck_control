#include "includes/crc.hpp"

int main(int argc, char *argv[]){
	CenterResiliencyCoordinator::CenterRC CRC;
	
	while(1){
		CRC.Communicate();
		usleep(30);  //cycle_time
	}

	return 0;
}
