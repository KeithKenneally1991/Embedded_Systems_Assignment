
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"
#include "lib/mmem.h"
#include "lib/list.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "Broadcast example");
AUTOSTART_PROCESSES(&example_broadcast_process);
/*---------------------------------------------------------------------------*/


static struct mmem mmem;
static void
 test_mmem(void)
 {
   struct my_struct {
     int a;
   } 

   my_data, *my_data_ptr;
	
 
   if(mmem_alloc(&mmem, sizeof(my_data)) == 0) {
     printf("memory allocation failed\n");
   } else {
     printf("memory allocation succeeded\n");
     my_data.a = 0xaa;
     memcpy(MMEM_PTR(&mmem), &my_data, sizeof(my_data));
     /* The cast below is safe only if the struct is packed */
     my_data_ptr = (struct my_struct *)MMEM_PTR(&mmem);
     printf("Value a equals 0x%x\n", my_data_ptr->a);
     mmem_free(&mmem);
   }
 }

struct example_list_struct {
  struct n *next;
  int number;
  //char neighborName ;
};

LIST(example_list);
static struct example_list_struct element1, element2;

void
//example_function(rimeaddr_t *dest, const rimeaddr_t *from)
example_function(int n, int m)
{
 struct example_list_struct *s;


	//uint8_t i;
  	//for(i = 0; i < RIMEADDR_SIZE; i++) {
   	// dest->u8[i] = src->u8[i];
  	 //}
	

//char str[] = {'m', 'n'};
	printf("m is %d",m );
	printf("n is %d",n );
	//int f = m +"."+ n;
	//printf("float is %s",str );
  //String s = "Dog";
	//printf("%s",greeting);
  list_init(example_list);

 // element1.neighborName = 1;
  element1.number = m;
  list_add(example_list, &element1);

  //element2.number = n;
  //list_add(example_list, &element2);

    printf("message recieved from node: ");
  for(s = list_head(example_list);
      s != NULL;
      s = list_item_next(s)) {
    //printf("message recieved from node:  %d\n", s->number);
    printf(" %d\n.", s->number);
  }
}



//------------------------------------
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	example_function(from->u8[0], from->u8[1]);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(example_broadcast_process, ev, data)
{
  SENSORS_ACTIVATE(button_sensor);

//static struct etimer et;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call);

  while(1) {

    /* Delay 2-4 seconds */
    //etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));

    PROCESS_WAIT_EVENT_UNTIL( (ev==sensors_event) && (data== &button_sensor) );

    packetbuf_copyfrom("King Keith", 10);
    broadcast_send(&broadcast);
    printf("broadcast message sent\n");
    //test_mmem();


  }

  PROCESS_END();   
 
}

//-------------------------------------------------------------------


 







