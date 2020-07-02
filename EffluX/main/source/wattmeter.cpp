#include "../inc/wattmeter.h"
#define DEBUG_WATTMETER
ADE7953 myADE7953(22, 21);

wattmeter_data wattmeter_get() {
	wattmeter_data wattdata;
	wattdata.apnoload = myADE7953.getAPNOLOAD();
	#ifdef DEBUG_WATTMETER
		Serial.print("APNOLOAD (hex): ");
		Serial.println(apnoload, HEX);
	#endif
	delay(200);

	wattdata.vRMS = myADE7953.getVrms();
	#ifdef DEBUG_WATTMETER
		Serial.print("Vrms (V): ");
		Serial.println(vRMS);
	#endif
	delay(200);

	wattdata.iRMSA = myADE7953.getIrmsA();
	#ifdef DEBUG_WATTMETER
		Serial.print("IrmsA (mA): ");
		Serial.println(iRMSA);
	#endif
	delay(200);

	wattdata.apparentPowerA = myADE7953.getInstApparentPowerA();
	#ifdef DEBUG_WATTMETER
		Serial.print("Apparent Power A (mW): ");
		Serial.println(apparentPowerA);
	#endif
	delay(200);

	wattdata.activePowerA = myADE7953.getInstActivePowerA();
	#ifdef DEBUG_WATTMETER
		Serial.print("Active Power A (mW): ");
		Serial.println(activePowerA);
	#endif
	delay(200);

	wattdata.reactivePowerA = myADE7953.getInstReactivePowerA();
	#ifdef DEBUG_WATTMETER
		Serial.print("Rective Power A (mW): ");
		Serial.println(reactivePowerA);
	#endif
	delay(200);

	wattdata.powerFactorA = myADE7953.getPowerFactorA();
	#ifdef DEBUG_WATTMETER
		Serial.print("Power Factor A (x100): ");
		Serial.println(powerFactorA);
	#endif
	delay(200);

	wattdata.activeEnergyA = myADE7953.getActiveEnergyA();
	#ifdef DEBUG_WATTMETER
		Serial.print("Active Energy A (hex): ");
		Serial.println(activeEnergyA);
	#endif
	delay(200);

	#ifdef DEBUG_WATTMETER
		Serial.println();
	#endif

	return wattdata;
}
