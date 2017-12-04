  // latest version
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "/home/user/contiki-3.0/dev/cc2420/cc2420.h"
#include "dev/button-sensor.h"
#include "lib/memb.h"
#include "dev/leds.h"
#include "lib/mmem.h"
#include "lib/list.h"
#include "string.h"

//#include "dev/sensinode-sensors.h"
#include "lib/sensors.h"
#include "dev/sht11/sht11-sensor.h"
//#include "dev/sht11-sensor.h"
#include "dev/temperature-sensor.h"
//#include "board.h"

#include <stdio.h>


/*
struct example_list_struct {
  struct n *next;
  int number;
  //char neighborName ;
};
*/

//LIST(example_list);
//static struct example_list_struct element1, element2;

// here

//global variables
  int hasBeenStored = 0;

      static int val;
struct broadcast_message {
  uint8_t seqno;
};

// list for sample temperatures
#define MAX_SAMPLES 5
struct temperatures {
       struct temperatures *next;
	int tempSample;
};
MEMB(temperatures_memb, struct temperatures, MAX_SAMPLES);
LIST(temperatures_list);
 

/* This structure holds information about neighbors. */
struct neighbor {
  /* The ->next pointer is needed since we are placing these on a
     Contiki list. */
  struct neighbor *next;

  /* The ->addr field holds the Rime address of the neighbor. */
  linkaddr_t addr;

  /* The ->last_rssi and ->last_lqi fields hold the Received Signal
     Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
     values that are received for the incoming broadcast packets. */
  uint16_t last_rssi, last_lqi;

  /* Each broadcast packet contains a sequence number (seqno). The
     ->last_seqno field holds the last sequenuce number we saw from
     this neighbor. */
  uint8_t last_seqno;

  /* The ->avg_gap contains the average seqno gap that we have seen
     from this neighbor. */
  //uint32_t avg_seqno_gap;

};

/* This #define defines the maximum amount of neighbors we can remember. */
#define MAX_NEIGHBORS 16

/* This MEMB() definition defines a memory pool from which we allocate
   neighbor entries. */
MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);

/* The neighbors_list is a Contiki list that holds the neighbors we
   have seen thus far. */
LIST(neighbors_list);

/* These hold the broadcast and unicast structures, respectively. */
static struct broadcast_conn broadcast;
//static struct unicast_conn unicast;

#define SEQNO_EWMA_UNITY 0x100
#define SEQNO_EWMA_ALPHA 0x040

PROCESS(example_unicast_process, " sending a temp ");
PROCESS(example_broadcast_process, "Broadcast example");
PROCESS(temp_humidity_process,"Sensors Example");
//AUTOSTART_PROCESSES(&example_broadcast_process, &temp_humidity_process, &example_unicast_process);
AUTOSTART_PROCESSES(&example_broadcast_process, &temp_humidity_process);



/*---------------------------------------------------------------------------*/
// 			UNICAST CODE
//---------------------------------------------------------------------------
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("\nUnicast message received from %d.%d\n",
	 from->u8[0], from->u8[1]);
 // char *res = (char *)packetbuf_dataptr();
   
      printf("\nThe message is ===> %s\n ",(char *)packetbuf_dataptr());
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

PROCESS_THREAD(example_unicast_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
    
  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);

  //printf("\nin the unicast ");

  while(1) {
  printf("LOOP once");
    static struct etimer et;
    linkaddr_t addr;
    int x = 33;
    char input[] = "55"; 
    etimer_set(&et, CLOCK_SECOND);
    
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
   // packetbuf_copyfrom("Hello", 6);
    packetbuf_copyfrom(input , 3);

    addr.u8[0] = 175;          // 175
    addr.u8[1] = 135;            // 135

    //if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      unicast_send(&uc, &addr);
    break;
    //}

  }

  PROCESS_END();
}
// ----------------------------------------------------------------
//  			END OF UNICAST
//   ------------------------------------------------------------------- 

/*
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

	//example_function(from->u8[0], from->u8[1]);
}
*/

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{

  static signed char rss;
  static signed char rss_val;
  static signed char rss_rssi;
  static signed char rss_offset;
   // rss_offset=-45;
  rss_val = cc2420_last_rssi;
  rss=rss_val+rss_offset;
  printf("RSSI in dBm is now %d ", rss);   

  hasBeenStored = 0;  // reset this flag everytime a new broadcast has been recieved
  struct neighbor *n;
  struct broadcast_message *m;
  uint8_t seqno_gap;

  /* The packetbuf_dataptr() returns a pointer to the first data byte
     in the received packet. */
  m = packetbuf_dataptr();

  /* Check if we already know this neighbor. */
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
	//printf("\nChecking for match..");
    /* We break out of the loop if the address of the neighbor matches
       the address of the neighbor from which we received this
       broadcast message. */
    if(linkaddr_cmp(&n->addr, from)) {
	//printf("\nmatch found..\n");  
	hasBeenStored = 1;    
	break;
    }
  }

  /* If n is NULL, this neighbor was not found in our list, and we
     allocate a new struct neighbor from the neighbors_memb memory
     pool. */

  if(n == NULL) {
    n = memb_alloc(&neighbors_memb);

    /* If we could not allocate a new neighbor entry, we give up. We
       could have reused an old neighbor entry, but we do not do this
       for now. */
    if(n == NULL) {
       return;
    }

    /* Initialize the fields. */
    linkaddr_copy(&n->addr, from);

   // n->last_seqno = m->seqno - 1;
   // n->avg_seqno_gap = SEQNO_EWMA_UNITY;

    /* Place the neighbor on the neighbor list. */
    list_add(neighbors_list, n);
  }

  /* We can now fill in the fields in our neighbor entry. */
     
   n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

   n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);


  /* Compute the average sequence number gap we have seen from this neighbor. */
 /* seqno_gap = m->seqno - n->last_seqno;
  n->avg_seqno_gap = (((uint32_t)seqno_gap * SEQNO_EWMA_UNITY) *
                      SEQNO_EWMA_ALPHA) / SEQNO_EWMA_UNITY +
                      ((uint32_t)n->avg_seqno_gap * (SEQNO_EWMA_UNITY -
                                                     SEQNO_EWMA_ALPHA)) /
    SEQNO_EWMA_UNITY;
*/
  /* Remember last seqno we heard. */
 // n->last_seqno = m->seqno;


  //printf("broadcast message received from %d.%d with seqno %d, RSSI %u, LQI %u, avg seqno gap %d.%02d\n",
       printf("\nbroadcast message received from %d.%d , RSSI %u, LQI %u\n",
	 from->u8[0], from->u8[1],
         packetbuf_attr(PACKETBUF_ATTR_RSSI),           // get the RSSI value
         packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY));  // get the LQI 

      // printf("\nMessage recieved is %s ",(char *)packetbuf_dataptr());

         
	if(hasBeenStored == 0){ // if this global variable is currently set 0 then this is a new neighbor found
		printf("New neighbor added to list.\n");
	}
	else{
		printf("Not added: Neighbor already exists.\n"); // if not 0, then thhis neighbor already exists in the list
	}

// try print out all elements of the list
printf("\nWe just added the address: %d", n->addr);
for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
	//printf("\nChecking for match..");
	//printf("LIst elements: %d", n->addr);
	printf("rssi values stored: %d", n->last_rssi);
    
    
  }
    // printf("RSSI in dBm is now %d, " rss);    


}// ends here

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/

	int counter = 0;               // A COUNTER USED IN GETTING THE AVERAGES
	int averageTemp = 0;    // HOLD THE AVERAGE TEMPERATURE
	int averageHumidity = 0;  // HOLD THE AVERAGE HUMIDITY

PROCESS_THREAD(temp_humidity_process, ev, data)
{
  struct temperatures *n;
      static struct etimer et;
    
     PROCESS_BEGIN();

    while(1) {  

         etimer_set(&et, CLOCK_SECOND * 3);
      	 SENSORS_ACTIVATE(sht11_sensor);
        
	 PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  	int16_t temp = ((sht11_sensor.value(SHT11_SENSOR_TEMP) / 10.0) - 396.0) / 10.0;
 	uint16_t rawHumidity = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
 	float ss = (((0.0405*rawHumidity) - 4) + ((-2.8 * 0.000001)*(pow(rawHumidity,2))));
 	int16_t humidity_true = (temp - 25) * (0.01 + 0.00008*rawHumidity) + ss;

   
     printf("\nTemperature: %d C", temp);
     printf(" <<<<>>>>>  Humidity: %d%\n", humidity_true);

	averageTemp += temp;
	averageHumidity += humidity_true;
	counter++;
	printf("\n counter is now %d ", counter);
if(counter == 5){
	averageTemp = averageTemp / 5;
	averageHumidity = averageHumidity / 5;
	printf("\n ------------------------------------------------\nThe average temp is %d ", averageTemp);
	printf("\n ------------------------------------------------\nThe average humidity is %d ", averageHumidity);


	// SEND A UNICAST TO NEIGHBOR WITH AVERAGES 
 	 unicast_open(&uc, 146, &unicast_callbacks);


 	 printf("\nTrying to send a unicast...");
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

                addr.u8[0] = 175;          // 175
                addr.u8[1] = 135;            // 135

    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
    		unicast_send(&uc, &addr);
	}	
   	       // packetbuf_copyfrom(humidMsg , 32);
     		//unicast_send(&uc, &addr);
   	        //packetbuf_copyfrom(humidstr , 3);
	// THEN RESET THE AVERAGES

	averageTemp = 0;
	averageHumidity = 0;	
	counter = 0;
//foundAverage = 0;


}
	  
if(n == NULL) {
    n = list_item_next(n);
	linkaddr_copy(&n->tempSample, temp);

    	list_add(temperatures_list, n);
  	n = memb_alloc(&temperatures_memb);
       
       } // if statement
  n->tempSample = temp; 
        	//printf("tempareture stored is %d", n->tempSample);

	}// ??


		printf("List: ");
     for(n=list_head(temperatures_list);n!=NULL; n=list_item_next(n)){

             //list_remove(lista_list, n);
			//printf("%d", n-> tempSample);

             //memb_free(&lista_memb, n);

     }

    PROCESS_END();

} // close temp/humid thread


PROCESS_THREAD(example_broadcast_process, ev, data)
{
  
  SENSORS_ACTIVATE(button_sensor);

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  broadcast_open(&broadcast, 129, &broadcast_call);
  PROCESS_BEGIN();

  while(1) {   

     PROCESS_WAIT_EVENT_UNTIL( (ev==sensors_event) && (data== &button_sensor) );
     packetbuf_copyfrom("King Keith", 10);
     broadcast_send(&broadcast);
     printf("broadcast message sent\n");
    
  }

  PROCESS_END();   
 
}

//-------------------------------------------------------------------


 








