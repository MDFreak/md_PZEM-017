# md_PZEM-017   
Arduino communication library modified 
  from Peacefair PZEM-017 v1.0 Energy monitor
    slightly modify from Jakub Mandula's PZEM-004T-v30 library (https://github.com/mandulaj/PZEM-004T-v30)
***
### hardware and connections
PZEM-003      - DC current/energy measurement (150V, 10A)
PZEM-017      - DC current/energy measurement (150V, 50A/100A)  
MAX485 module - used for communication between ESP32 - PZEM-0xx 
#### pin connection to ESP
	MAX485 	ESP32   
	RO			UART2-RxD (16) - receiver out
	RE-DE   
	    

#### connection to load

Make sure the device is connected to the 5v power!
	



```
# Installation instructions
You should be able to install the library from the Library Manager in the Arduino IDE. You can also download the ZIP of this repository and install it manually. A guide on how to do that is over here: [https://www.arduino.cc/en/guide/libraries](https://www.arduino.cc/en/guide/libraries) 

***
Thank you to [@olehs](https://github.com/olehs) and [Jakub Mandula](https://github.com/mandulaj) for this great library.
