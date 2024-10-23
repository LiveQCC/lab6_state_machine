#include <stdio.h>
#include <stdlib.h>
#include <esp32/rom/ets_sys.h>
#include <esp_task_wdt.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_PIN_LEVEL_UP 12
#define LED_PIN_LEVEL_MIDDLE 14
#define LED_PIN_LEVEL_DOWN 27
#define BUTTON_PIN 26
#define ELEVATOR_IDLE 0
#define ELEVATOR_MOVING 1
#define ELEVATOR_LOADING 2
#define LEVEL_UPPER 2
#define LEVEL_MIDDLE 1
#define LEVEL_LOWER 0


#define PUSH_TIME_US 250000 // 250 ms

// Used to represent a travel need of a passenger.
struct travel_need {
    int origin;
    int destination;
};
struct elevator_state {
    int current_level;
    int state;
    int travel_need_index;
};

static struct elevator_state elevator = {LEVEL_LOWER, ELEVATOR_IDLE, -1};

// Used to not allow button pushes that are too close to each other in time
static volatile uint64_t lastPush = -PUSH_TIME_US;

//Just a counter keeping track of which travel need is next to process.
static volatile int travel_need_counter = 0;

// This data structure holds information about 
static volatile struct travel_need travel_needs[50];

// This function is called when button is pushed
static void handle_push(void *arg) {
    // Disable interrupts
    gpio_intr_disable(BUTTON_PIN);

    // Get the current time 
    uint64_t now = esp_timer_get_time();

    // If enough time passed, we should consider this event as a genuine push
    if ((now - lastPush) > PUSH_TIME_US) {
        lastPush = now;
        printf("Button pushed\n");
        if(elevator.current_level == LEVEL_LOWER){
            gpio_set_level(LED_PIN_LEVEL_MIDDLE, 0);
            gpio_set_level(LED_PIN_LEVEL_DOWN, 1);
            gpio_set_level(LED_PIN_LEVEL_UP, 0);
        }
        else if(elevator.current_level == LEVEL_MIDDLE){
            gpio_set_level(LED_PIN_LEVEL_MIDDLE, 1);
            gpio_set_level(LED_PIN_LEVEL_DOWN, 0);
            gpio_set_level(LED_PIN_LEVEL_UP, 0);
        }
        else if(elevator.current_level == LEVEL_UPPER){
            gpio_set_level(LED_PIN_LEVEL_MIDDLE, 0);
            gpio_set_level(LED_PIN_LEVEL_DOWN, 0);
            gpio_set_level(LED_PIN_LEVEL_UP, 1);
        }
        // If the elevator is idle, assign the next travel need
        if (elevator.state == ELEVATOR_IDLE) {
            elevator.travel_need_index = travel_need_counter;
            travel_need_counter++;
            elevator.state = ELEVATOR_LOADING;
        }
    }

    // Re-enable interrupts
    gpio_intr_enable(BUTTON_PIN);
}
    
    void lala(){
/*
         if (elevator.state == ELEVATOR_LOADING) {
            // Simulate loading time
       //     vTaskDelay(5000 / portTICK_PERIOD_MS);

            // Move the elevator to the destination level
            struct travel_need current_travel_need = travel_needs[elevator.travel_need_index];
            int destination = current_travel_need.destination;

            while (elevator.current_level != destination) {
                if (elevator.current_level < destination) {
                   // wait 
                    elevator.current_level++;
                } else {
                    elevator.current_level--;
                }

                // Simulate travel time
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }

            // Simulate unloading time
            vTaskDelay(5000 / portTICK_PERIOD_MS);

            // Set elevator to idle
            elevator.state = ELEVATOR_IDLE;
        }*/
    }
void app_main() {

    //Initialize travel needs (50 randomly generated travel needs)
    travel_needs[0].origin = 2; travel_needs[0].destination = 1;
    travel_needs[1].origin = 1; travel_needs[1].destination = 2;
    travel_needs[2].origin = 1; travel_needs[2].destination = 2;
    travel_needs[3].origin = 0; travel_needs[3].destination = 2;
    travel_needs[4].origin = 2; travel_needs[4].destination = 1;
    travel_needs[5].origin = 0; travel_needs[5].destination = 2;
    travel_needs[6].origin = 1; travel_needs[6].destination = 2;
    travel_needs[7].origin = 1; travel_needs[7].destination = 0;
    travel_needs[8].origin = 0; travel_needs[8].destination = 1;
    travel_needs[9].origin = 1; travel_needs[9].destination = 0;
    travel_needs[10].origin = 1; travel_needs[10].destination = 2;
    travel_needs[11].origin = 0; travel_needs[11].destination = 1;
    travel_needs[12].origin = 0; travel_needs[12].destination = 2;
    travel_needs[13].origin = 0; travel_needs[13].destination = 1;
    travel_needs[14].origin = 0; travel_needs[14].destination = 2;
    travel_needs[15].origin = 0; travel_needs[15].destination = 1;
    travel_needs[16].origin = 2; travel_needs[16].destination = 1;
    travel_needs[17].origin = 2; travel_needs[17].destination = 1;
    travel_needs[18].origin = 1; travel_needs[18].destination = 0;
    travel_needs[19].origin = 2; travel_needs[19].destination = 1;
    travel_needs[20].origin = 1; travel_needs[20].destination = 0;
    travel_needs[21].origin = 0; travel_needs[21].destination = 1;
    travel_needs[22].origin = 1; travel_needs[22].destination = 2;
    travel_needs[23].origin = 0; travel_needs[23].destination = 2;
    travel_needs[24].origin = 2; travel_needs[24].destination = 1;
    travel_needs[25].origin = 1; travel_needs[25].destination = 0;
    travel_needs[26].origin = 1; travel_needs[26].destination = 2;
    travel_needs[27].origin = 0; travel_needs[27].destination = 2;
    travel_needs[28].origin = 1; travel_needs[28].destination = 0;
    travel_needs[29].origin = 1; travel_needs[29].destination = 2;
    travel_needs[30].origin = 0; travel_needs[30].destination = 1;
    travel_needs[31].origin = 1; travel_needs[31].destination = 2;
    travel_needs[32].origin = 0; travel_needs[32].destination = 2;
    travel_needs[33].origin = 0; travel_needs[33].destination = 2;
    travel_needs[34].origin = 1; travel_needs[34].destination = 2;
    travel_needs[35].origin = 2; travel_needs[35].destination = 1;
    travel_needs[36].origin = 0; travel_needs[36].destination = 2;
    travel_needs[37].origin = 1; travel_needs[37].destination = 0;
    travel_needs[38].origin = 0; travel_needs[38].destination = 2;
    travel_needs[39].origin = 2; travel_needs[39].destination = 1;
    travel_needs[40].origin = 0; travel_needs[40].destination = 1;
    travel_needs[41].origin = 0; travel_needs[41].destination = 1;
    travel_needs[42].origin = 0; travel_needs[42].destination = 1;
    travel_needs[43].origin = 1; travel_needs[43].destination = 0;
    travel_needs[44].origin = 0; travel_needs[44].destination = 2;
    travel_needs[45].origin = 2; travel_needs[45].destination = 1;
    travel_needs[46].origin = 2; travel_needs[46].destination = 1;
    travel_needs[47].origin = 2; travel_needs[47].destination = 1;
    travel_needs[48].origin = 0; travel_needs[48].destination = 2;
    travel_needs[49].origin = 1; travel_needs[49].destination = 0;



     gpio_config_t config;
    // Configure pins as output
    config.pin_bit_mask = (1ULL << LED_PIN_LEVEL_UP) | (1ULL << LED_PIN_LEVEL_MIDDLE) | (1ULL << LED_PIN_LEVEL_DOWN);
    config.mode = GPIO_MODE_OUTPUT;
    gpio_config(&config);

    // Configure button pin as input with pull-up and interrupt
    config.pin_bit_mask = (1ULL << BUTTON_PIN);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&config);

    // Activate the interrupts for the GPIOs
    //gpio_install_isr_service(0);
    //gpio_isr_handler_add(BUTTON_PIN, handle_push, NULL);
    // Activate the interrupts for the GPIOs
    gpio_install_isr_service(0);


   

    // Add a handler to the ISR for pin BUTTON_PIN
     gpio_isr_handler_add(BUTTON_PIN, handle_push, NULL);
  

    // This is where you most likely put your main elevator code. 
    while (1) {
       lala();
    }
      
  
       
}