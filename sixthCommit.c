  // Keith Kenneally
  // CS3
  // R00142850

#include "contiki.h"
#include "net/rime/rime.h"
#include "/home/user/contiki-3.0/dev/cc2420/cc2420.h"
#include "dev/button-sensor.h"
#include "lib/memb.h"
#include "core/dev/leds.h"
#include "lib/list.h"
#include "string.h"
#include "lib/sensors.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/temperature-sensor.h"
#include <stdio.h>


//global variables
int hasBeenStored = 0;

static int *highestRSSI;
static int *tempRSSI;

static linkaddr_t *sendUnicastAddr;       // store the address of the node with highest RSSI

//int isThisLastInChain = 0;

/* This structure holds information about neighbors. */
struct neighbor {
  
  struct neighbor *next; // pointer needed for the list

  linkaddr_t addr;   // holds the Rime address

  uint16_t last_rssi, last_lqi; //hold the Received Signal Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
  
  uint8_t last_seqno; // the last sequence number from a neighbor
 
};

#define MAX_NEIGHBORS 16      // max number of neighbors

/* This MEMB() definition defines a memory pool from which we allocate
   neighbor entries. */
MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);

/* The neighbors_list is a Contiki list that holds the neighbors we
   have seen thus far. */
LIST(neighbors_list);

/* These hold the broadcast and unicast structures, respectively. */
static struct broadcast_conn broadcast;

PROCESS(example_broadcast_process, "Broadcast example");
PROCESS(temp_humidity_process,"Sensors Example");
PROCESS (green_led_process, "Activate Green LED");
PROCESS (red_led_process, "Activate Red LED");
AUTOSTART_PROCESSES(&example_broadcast_process, &temp_humidity_process);


/*---------------------------------------------------------------------------*/
// 			UNICAST RECIEVED FUNCTION
//---------------------------------------------------------------------------

static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("\n----------------------------------------------------------");
  printf("\nUnicast message received from %d.%d\n",
	 from->u8[0], from->u8[1]);
  // char *res = (char *)packetbuf_dataptr();
   
        printf("The message is ===> %s\n",(char *)packetbuf_dataptr());
  printf("----------------------------------------------------------");
		
	process_start(&green_led_process, "Turn on green led"); 
        
      
     //isThisLastInChain = -1;
	// THE LAST MOTE ON THE CHAIN WILL ACTIVATE THIS RED LED
	//process_start(&red_led_process, "Turn on green led");
}

static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;


// ----------------------------------------------------------------
//  			TURN ON GREEN LED
//   ------------------------------------------------------------------- 

     PROCESS_THREAD(green_led_process, ev, data)
     {

	static struct etimer et;
        PROCESS_BEGIN();
          
       etimer_set(&et, CLOCK_SECOND * 3);              // TURN ON THE LED FOR 4 SECONDS
       leds_on(1); 				     // TURN ON THE LED
       PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // WAIT UNTIL THE TIMER ENDS
       leds_off(1);                                    // DEACTIVATE THE LED
   
       PROCESS_END();

     }

// ----------------------------------------------------------------
//  			TURN ON RED LED
//   ------------------------------------------------------------------- 

     PROCESS_THREAD(red_led_process, ev, data)
     {

       static struct etimer et;
       PROCESS_BEGIN();
           
      etimer_set(&et, CLOCK_SECOND * 4);	      // TURN ON THE LED FOR 4 SECONDS
      leds_on(4); 				     // TURN ON THE LED
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  // WAIT UNTIL THE TIMER ENDS
      leds_off(4);                                    // DEACTIVATE THE LED
     
      PROCESS_END();

     }


// ----------------------------------------------------------------
//  			BROADCAST RECIEVED FUNCTION
//   ------------------------------------------------------------------- 

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
 
  hasBeenStored = 0;  // reset this flag everytime a new broadcast has been recieved
  struct neighbor *n;
  struct broadcast_message *m;
  uint8_t seqno_gap;

  /* The packetbuf_dataptr() returns a pointer to the first data byte
     in the received packet. */
  m = packetbuf_dataptr();

  /* Check if we already know this neighbor. */
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {

    /* We break out of the loop if the address of the neighbor matches
       the address of the neighbor from which we received this
       broadcast message. */
	 
    if(linkaddr_cmp(&n->addr, from)) {  

       n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);   // THIS NEIGHBOR ALREADY EXISTS BUT STILL UPDATE THE RSSI VALUE
	hasBeenStored = 1;    
	break;
    }
  }
  
  if(n == NULL) {       //allocate a new struct neighbor from the neighbors_memb memory
    n = memb_alloc(&neighbors_memb);
  
    if(n == NULL) {
       return;
    }

    /* Initialize the fields. */
    linkaddr_copy(&n->addr, from);
  
    /* Place the neighbor on the neighbor list. */
    list_add(neighbors_list, n);
  }

        /* fill in the fields in the neighbor entry. */    
      n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
      n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
  printf("\n----------------------------------------------------------");
       printf("\nBroadcast message received from %d.%d , RSSI %d, LQI %d\n",
	 from->u8[0], from->u8[1],     
         packetbuf_attr(PACKETBUF_ATTR_RSSI),           // get the RSSI value
         packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY));  // get the LQI 

      // printf("\nMessage recieved is %s ",(char *)packetbuf_dataptr());
     
	if(hasBeenStored == 0){ // if this global variable is currently set 0 then this is a new neighbor found
		printf("New neighbor added to list.");
	}
	else{
		printf("Not added: Neighbor already exists."); // if not 0, then this neighbor already exists in the list
	}
   printf("\n----------------------------------------------------------\n");
        
        // FIND THE NODE WITH THE HIGHEST RSSI
	for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) { 
           //printf("sequesnce number is %d ", n->last_seqno);   
      		 tempRSSI = n->last_rssi;
        	
   		 if(tempRSSI >= highestRSSI){ 

		    highestRSSI = tempRSSI;  
		    sendUnicastAddr = &n->addr;   // THIS GLOBAL VARIABLE HOLDS THE ADDRESS OF THE NODE WITH HIGHEST RSSI
	         }       
        }  
               

}// ends here

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;



// ------------------------------------------------------------------------
//  	 GET TEMPERATURE/HUMIIDTY/AVERAGES PROCESS --> SEND UNICAST MESSAGE
//   ---------------------------------------------------------------------- 

// these variables are needed calculating the averages
int counter = 0;                // A COUNTER USED IN GETTING THE AVERAGES
int averageTemp = 0;           // HOLD THE AVERAGE TEMPERATURE
int averageHumidity = 0;       // HOLD THE AVERAGE HUMIDITY

PROCESS_THREAD(temp_humidity_process, ev, data)
{
      static struct etimer et;
    
      PROCESS_BEGIN();

    while(1) {  

         etimer_set(&et, CLOCK_SECOND * 6);
      	 SENSORS_ACTIVATE(sht11_sensor);
        
	 PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

        // FORMULA NEEDED TO GET THE TRUE TEMPERATURE AND HUMIDITY VALUES
  	int16_t temp = ((sht11_sensor.value(SHT11_SENSOR_TEMP) / 10.0) - 396.0) / 10.0;
 	uint16_t rawHumidity = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
 	float ss = (((0.0405*rawHumidity) - 4) + ((-2.8 * 0.000001)*(pow(rawHumidity,2))));
 	int16_t humidity_true = (temp - 25) * (0.01 + 0.00008*rawHumidity) + ss;
  
        printf("\nTemperature: %d C", temp);
        printf(" <<<<>>>>>  Humidity: %d%\n", humidity_true);

	averageTemp += temp;                     
	averageHumidity += humidity_true;
	counter++;

       // ---------------------------------------------------------------------------
       // IF THE COUNTER HITS 5 THEN CALCULATE THE AVERAGE AND SEND A UNICAST MESSAGE
       //----------------------------------------------------------------------------
       if(counter == 5){
	averageTemp = averageTemp / 5;
	averageHumidity = averageHumidity / 5;
	printf("\n ------------------------------------------------\nThe average temp is %d ", averageTemp);
	printf("The average humidity is %d \n ------------------------------------------------\n", averageHumidity);


	// THIS IS NEEDED TO SEND THE UNICAST MESSAGE BELOW
 	 unicast_open(&uc, 146, &unicast_callbacks);
   	 linkaddr_t addr;
	
		// ---------------------------------------------------------------------------
		// TO SEND THE AVERAGES, I WILL CONCATENATE BOTH AVERAGES INTO ONE LONG MESSAGE
		// ---------------------------------------------------------------------------
		char message[34] = "Avg temp: ";     // ORIGINAL MESSAGE
		char message2[22] = " degrees, Avg Humid: ";   // SECOND PART OF MESSAGE
			
		char tempstr[3];
		sprintf(tempstr, "%d", averageTemp);     // CONVERT THE TEMP AVG TO A STRING
		char humidstr[3];
		sprintf(humidstr, "%d", averageHumidity);  // CONVERT THE TEMP HUMIDIDY TO A STRING
		
		// PUT IT ALL TOGETHER
		strcat(message, tempstr);
		strcat(message, message2);
		strcat(message, humidstr);
   	        packetbuf_copyfrom(message , 35);            
		
                // THE UNICAST WILL BE SENT TO THIS ADDRESS, WHICH IS THE ADDRESS OF THE HIGHEST RSSI
                addr.u8[0] = sendUnicastAddr->u8[0]; 
		addr.u8[1] = sendUnicastAddr->u8[1];
        	
                printf("\nTrying to send a unicast containing message: %s ", message);
    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
    				
                unicast_send(&uc, &addr);	
 	        printf("\nUnicast sent to address of  %d.%d ", sendUnicastAddr->u8[0], sendUnicastAddr->u8[1]);
            
	}	
   	       
	// THEN RESET THE AVERAGES AND COUNTER FOR THE NEXT ITERATION
	averageTemp = 0;
	averageHumidity = 0;	
	counter = 0;
}

}
    PROCESS_END();

} // close temp/humid thread


// ------------------------------------------------------------------------
//  	                  SEND A BRODACAST
//   ---------------------------------------------------------------------- 
PROCESS_THREAD(example_broadcast_process, ev, data)
{
  
  SENSORS_ACTIVATE(button_sensor);

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  broadcast_open(&broadcast, 129, &broadcast_call);
  PROCESS_BEGIN();

  while(1) {   

     PROCESS_WAIT_EVENT_UNTIL( (ev==sensors_event) && (data== &button_sensor) );  // WAIT UNTILL THE BUTTON IS PRESSED
     packetbuf_copyfrom("Keith", 6);
     broadcast_send(&broadcast);
     printf("Broadcast message sent\n");
    
  }
  PROCESS_END();   
}


