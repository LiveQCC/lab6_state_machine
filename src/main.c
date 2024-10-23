#include <stdio.h>
#include <stdlib.h>
#include <esp32/rom/ets_sys.h>
#include <esp_task_wdt.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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
#define MAX_QUEUE_SIZE 50

struct travel_need {
    int origin;
    int destination;
};

struct elevator_state {
    int current_level;
    int state;
    int current_index;
};
/*
    
    Initiera en struct med nivå 0 och state idle.
    current index -1 (ingen request)
*/
static struct elevator_state elevator = {
    .current_level = LEVEL_LOWER,
    .state = ELEVATOR_IDLE,
    .current_index = -1
};

static volatile uint64_t lastPush = -PUSH_TIME_US;
static volatile int travel_need_counter = 0;
static struct travel_need travel_needs[50];  // Your original travel needs array
static TaskHandle_t elevator_task_handle = NULL;
static int arrivedAtOrigin = 0;

// skapa FREE RTOS QUEUE
static QueueHandle_t travel_queue = NULL;

void update_led_status() {
    if(elevator.current_level == LEVEL_LOWER) {
        gpio_set_level(LED_PIN_LEVEL_MIDDLE, 0);
        gpio_set_level(LED_PIN_LEVEL_DOWN, 1);
        gpio_set_level(LED_PIN_LEVEL_UP, 0);
    }
    else if(elevator.current_level == LEVEL_MIDDLE) {
        gpio_set_level(LED_PIN_LEVEL_MIDDLE, 1);
        gpio_set_level(LED_PIN_LEVEL_DOWN, 0);
        gpio_set_level(LED_PIN_LEVEL_UP, 0);
    }
    else if(elevator.current_level == LEVEL_UPPER) {
        gpio_set_level(LED_PIN_LEVEL_MIDDLE, 0);
        gpio_set_level(LED_PIN_LEVEL_DOWN, 0);
        gpio_set_level(LED_PIN_LEVEL_UP, 1);
    }
}

static void IRAM_ATTR handle_push(void *arg) {
    gpio_intr_disable(BUTTON_PIN);
    uint64_t now = esp_timer_get_time();
    
    if ((now - lastPush) > PUSH_TIME_US) {
        lastPush = now;
        
        // Ingen notifikation behövs, vi kollar bara om knappen har tryckts
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Elevator task ska köras = priority set to true
        vTaskNotifyGiveFromISR(elevator_task_handle, &xHigherPriorityTaskWoken);
        // Om en högre prioritet finns, yield och låt den köras
        // Kör elevator task
        //  https://www.freertos.org/Documentation/02-Kernel/04-API-references/05-Direct-to-task-notifications/02-vTaskNotifyGiveFromISR <- Om du glömmer
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
    
    gpio_intr_enable(BUTTON_PIN);
}

void elevator_task(void *pvParameters) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    struct travel_need current_travel_need;
    
    while (1) {
        esp_task_wdt_reset();
        
        // kollar om knappen har tryckts med notifikation. om det inte finns retunerar den 0 direkt. Finnns det lägg till i kön given från rad 83
        if (ulTaskNotifyTake(pdTRUE, 0) != 0) {
            if (travel_need_counter < 50) {
                // kollar om det finns plats i kön, lägger till i så fall
                struct travel_need new_need = travel_needs[travel_need_counter];
                if (xQueueSend(travel_queue, &new_need, 0) == pdTRUE) {
                    printf("Added to queue: Origin %d, Destination %d\n", 
                           new_need.origin, new_need.destination);
                    travel_need_counter++;
                    // Om vi är i idle, börja processa
                    if (elevator.state == ELEVATOR_IDLE) {
                        arrivedAtOrigin = 0;
                        elevator.state = ELEVATOR_LOADING;
                    }
                }
            }
        }
        
        if (elevator.state == ELEVATOR_LOADING) {
            // om vi inte har något att göra, gå till idle
            if (arrivedAtOrigin == 0) {
                // kolla om det finns något i the queue, if not idle och continue
                if (xQueuePeek(travel_queue, &current_travel_need, 0) != pdTRUE) {
                    elevator.state = ELEVATOR_IDLE;
                    continue;
                }
            }
            
            int destination = current_travel_need.destination;
            int origin = current_travel_need.origin;
            
            // Om vi inte är på origin, åk dit
            if (elevator.current_level != origin && arrivedAtOrigin == 0) {
                printf("Elevator moving to origin %d\n", origin);
                elevator.state = ELEVATOR_MOVING;
                
                if (elevator.current_level < origin) {
                    elevator.current_level++;
                } else {
                    elevator.current_level--;
                }
                
                vTaskDelay(pdMS_TO_TICKS(3000));
                update_led_status();
                elevator.state = ELEVATOR_LOADING;
            } else {
                arrivedAtOrigin = 1;
            }
            
            // Om vi inte är på destinationen, åk dit ( så länge vi har varit på origin)
            if (elevator.current_level != destination && arrivedAtOrigin == 1) {
                elevator.state = ELEVATOR_MOVING;
                
                if (elevator.current_level < destination) {
                    elevator.current_level++;
                } else {
                    elevator.current_level--;
                }
                
                vTaskDelay(pdMS_TO_TICKS(3000));
                update_led_status();
                elevator.state = ELEVATOR_LOADING;
            } else if (elevator.current_level == destination && arrivedAtOrigin == 1) {
                printf("Arrived at destination %d\n", destination);
                  vTaskDelay(pdMS_TO_TICKS(2000));
                xQueueReceive(travel_queue, &current_travel_need, 0); // Ta bort från kön
                arrivedAtOrigin = 0;
                
                // Check if queue is empty
                if (uxQueueMessagesWaiting(travel_queue) == 0) {
                    elevator.state = ELEVATOR_IDLE;
                    printf("Inga fler åkare\n");
                } else {
                    printf("Åker till nästa knapptryck!!!!\n");
                }
            }
        }
        
        update_led_status();
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void app_main() {

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
    // Create the FreeRTOS queue
    travel_queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(struct travel_need));
    if (travel_queue == NULL) {
        printf("Failed to create queue\n");
        return;
    }

    // GPIO configuration
    gpio_config_t config;
    config.pin_bit_mask = (1ULL << LED_PIN_LEVEL_UP) | (1ULL << LED_PIN_LEVEL_MIDDLE) | (1ULL << LED_PIN_LEVEL_DOWN);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
    
    // Button configuration
    config.pin_bit_mask = (1ULL << BUTTON_PIN);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&config);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, handle_push, NULL);
    
    // Create the elevator task
    xTaskCreate(elevator_task, "elevator_task", 2048, NULL, 5, &elevator_task_handle);
    
    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}