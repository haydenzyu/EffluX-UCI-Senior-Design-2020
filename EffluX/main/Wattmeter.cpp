#include "Wattmeter.h"

//#define local_SPI_freq 1000000  //Set SPI_Freq at 1MHz (#define, (no = or ;) helps to save memory)
//#define local_SS 14  //Set the SS pin for SPI communication as pin 10  (#define, (no = or ;) helps to save memory)

int Current = 0;
int Voltage = 0;
char t1[30];	
ADE7953 myADE7953(22, 21);

void WattmeterTest() {
	long apnoload, activeEnergyA;
	float vRMS, iRMSA, powerFactorA, apparentPowerA, reactivePowerA, activePowerA;
	getDateTimeNow(t1);
	apnoload = myADE7953.getAPNOLOAD();
	Serial.print("APNOLOAD (hex): ");
	Serial.println(apnoload, HEX);
	//send_data("APNOLOAD", apnoload, "hex", t1);
	delay(200);

	Serial.println(t1);
	vRMS = myADE7953.getVrms();
	Serial.print("Vrms (V): ");
	Serial.println(vRMS);
	send_data("Vrms", vRMS, "V", t1);
	delay(200);

	iRMSA = myADE7953.getIrmsA();
	Serial.print("IrmsA (mA): ");
	Serial.println(iRMSA);
	send_data("Irms", iRMSA, "mA", t1);
	delay(200);
	
	apparentPowerA = myADE7953.getInstApparentPowerA();
	Serial.print("Apparent Power A (mW): ");
	Serial.println(apparentPowerA);
	send_data("Apparent Power", apparentPowerA, "mW", t1);
	delay(200);

	activePowerA = myADE7953.getInstActivePowerA();
	Serial.print("Active Power A (mW): ");
	Serial.println(activePowerA);
	send_data("Active Power", activePowerA, "mW", t1);
	delay(200);
	
	reactivePowerA = myADE7953.getInstReactivePowerA();
	Serial.print("Rective Power A (mW): ");
	Serial.println(reactivePowerA);
	send_data("Reactive Power", reactivePowerA, "mW", t1);
	delay(200);

	powerFactorA = myADE7953.getPowerFactorA();
	Serial.print("Power Factor A (x100): ");
	Serial.println(powerFactorA);
	send_data("Power Factor", powerFactorA, "x100", t1);
	delay(200);

	activeEnergyA = myADE7953.getActiveEnergyA();
	Serial.print("Active Energy A (hex): ");
	Serial.println(activeEnergyA);
	//send_data("Active Energy", activeEnergyA, "hex", getDateTime());
	delay(200);

	Serial.println();
}
