#include <sdtio.h>

char rxbuff[11];
int16_t dacval1,dacval2;

main(){
	while(1){
		receive(rxbuff,11);
		if ((rxbuff[9]==0x0D) && (rxbuff[10]==0x0A)) {
			dacval1 = (rxbuff[0] - 48)*1000 + (rxbuff[1] - 48)*100 + \
								(rxbuff[2] - 48)*10 + (rxbuff[3] - 48);
			dacval2 = (rxbuff[5] - 48)*1000 + (rxbuff[6] - 48)*100 + \
								(rxbuff[7] - 48)*10 + (rxbuff[8] - 48);		
			dac_write(1,dacval1);
			dac_write(2,dacval2);
		}
	}
}	


