/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac_oneshot.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_random.h"
#include "tree_xmas.h"


  /* DAC oneshot init */
    dac_oneshot_handle_t chan0_handle;
    dac_oneshot_config_t chan0_cfg = {
        .chan_id = DAC_CHAN_0,
    };

    dac_oneshot_handle_t chan1_handle;
    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };

    adc_oneshot_unit_handle_t adc1_handle;
    uint32_t val = 0;

uint8_t status_en = 0;

uint32_t frame_counter = 0;

int point = 0;


int8_t anim_1(uint32_t point, const uint32_t point_nr_size)
{
  const uint32_t half_way = point_nr_size / 2;

  const uint32_t max_swing_size = point_nr_size;
  static uint32_t moving_point = 0;
    static uint8_t  ret_val = 0;


  /*
    if (point == 0) {
      
      if(moving_point < point_nr_size * 2)
      {
        moving_point+=4;
      }else
      {
        moving_point = 0;
      }
  }

if(moving_point < point_nr_size)
{
  if(point <= moving_point)
  {
    ret_val = 1;
  }else
  {
    ret_val = 0;
  }
}else
{
  if(point <= moving_point - point_nr_size)
  {
    ret_val = 0;
  }else
  {
    ret_val = 1;
  }
}

*/


static uint32_t frame = 0;

if(point == 0)
  frame++;

static uint32_t start_point = 0;
static uint32_t stop_point = 0;

if(point == 0)
{
  //increment every frame and wrap around
  if(start_point > point_nr_size)
  {
    start_point = 0;
  }else{
    start_point += 7;
  }

  if(stop_point > point_nr_size)
  {
    stop_point = 0;
  }else{
    stop_point++;
  }
}

if((point == start_point || point == stop_point) && (start_point != stop_point))
{
  if(ret_val)
    ret_val = 0;
  else
    ret_val = 1;
}

/*
if((point == start_point ))
{
  if(ret_val)
    ret_val = 0;
  else
    ret_val = 1;
}

if((point == stop_point))
{
  if(ret_val)
    ret_val = 0;
  else
    ret_val = 1;
}
*/


 return ret_val;

}

//scope attempt
uint8_t anim_2(uint8_t * out_x, uint8_t * out_y)
{
  #define FILT_TIME 40
  static uint8_t x = 0;
  static uint8_t y = 0;
  static uint8_t direction = 1; //1 - up, 0 - down
  uint8_t output = 0;


  static int32_t adc_val;
  static int32_t filtered_val[256];
  //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_val));
    output = 1;

  if(direction)
  {
    x++;
    if(x == 253) 
    {
       direction = 0;
 
       for(int i = 0; i < 256; i++)
       {
         ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_val));
         //limit values
         if(adc_val < (FILT_TIME+1)) adc_val = 4;
         if(adc_val > (4090 - FILT_TIME)) adc_val = (4090 - FILT_TIME);

         if(i==0){filtered_val[i] = adc_val;}
         else
         {
           if(adc_val > (filtered_val[i-1] + FILT_TIME)){filtered_val[i]  = filtered_val[i-1] + FILT_TIME;}
           else
           if(adc_val < (filtered_val[i-1] - FILT_TIME)){filtered_val[i]  = filtered_val[i-1] - FILT_TIME;}
           else
            {filtered_val[i] = adc_val;}
         }

         vTaskDelay(1 / portTICK_PERIOD_MS);
       }
    }
  }else
  {
    
    //x -= 15;
    //if(x < 25)
    x --;
    if(x == 0)
    {
       //x = 0;
       direction = 1;
       output = 1;
       for(int i = 0; i < 256; i++)
       {
         ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_val));
         //limit values
         if(adc_val < (FILT_TIME+1)) adc_val = 4;
         if(adc_val > (4090 - FILT_TIME)) adc_val = (4090 - FILT_TIME);

         if(i==0){filtered_val[i] = adc_val;}
         else
         {
           if(adc_val > (filtered_val[i-1] + FILT_TIME)){filtered_val[i]  = filtered_val[i-1] + FILT_TIME;}
           else
           if(adc_val < (filtered_val[i-1] - FILT_TIME)){filtered_val[i]  = filtered_val[i-1] - FILT_TIME;}
           else
            {filtered_val[i] = adc_val;}
         }

         vTaskDelay(1 / portTICK_PERIOD_MS);
       }
       //printf("%ld\n",filtered_val[200]);
    }
  }

  *out_x = x;
  //*out_y = adc_val[x]/16;
  //*out_y = filtered_val[x]/16;
  *out_y = filtered_val[x]/30;
  
  return output;
}

void timer_callback(void *param)
{
  

   //Enable for XMAS tree
  status_en = anim_1(point/2, SIZE_OF_XMAS3/2);

  ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan0_handle, tree3[point] >> 4)); //scale up x from 5 to 4
  ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan1_handle, tree3[point+1] >> 4));//scale up y from 5 to 4
  
    point+=2;

  if(point >= SIZE_OF_XMAS3 - 2)
  {
    point = 0;
    frame_counter++;
    //gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
  }
  

  //////////////////////////
  //Enable for scope
  //uint8_t x,y;
  //status_en = anim_2(&x, &y);
  //ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan0_handle, x));
  //ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan1_handle, y));
  
  //////////////////////////

  if(status_en == 1)
  {
    gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
  }else
  {
    gpio_set_level(GPIO_NUM_27, 0);
  }




/*
  //ON = !ON;
  if(ray_len == SIZE_OF_XMAS3)
  {
    ray_len = 0;
    //gpio_set_level(GPIO_NUM_27, 0);
  }
  else
  {
      status_en = 1;
    gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
  }
*/



}

// 20 - 200hz Single Pole Bandpass IIR Filter
float bassFilter(float sample) {
    static float xv[3] = {0,0,0}, yv[3] = {0,0,0};
    xv[0] = xv[1]; xv[1] = xv[2]; 
    xv[2] = sample / 9.1f;
    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] = (xv[2] - xv[0])
        + (-0.7960060012f * yv[0]) + (1.7903124146f * yv[1]);
    return yv[2];
}

// 10hz Single Pole Lowpass IIR Filter
float envelopeFilter(float sample) { //10hz low pass
    static float xv[2] = {0,0}, yv[2] = {0,0};
    xv[0] = xv[1]; 
    xv[1] = sample / 160.f;
    yv[0] = yv[1]; 
    yv[1] = (xv[0] + xv[1]) + (0.9875119299f * yv[0]);
    return yv[1];
}

// 1.7 - 3.0hz Single Pole Bandpass IIR Filter
float beatFilter(float sample) {
    static float xv[3] = {0,0,0}, yv[3] = {0,0,0};
    xv[0] = xv[1]; xv[1] = xv[2]; 
    xv[2] = sample / 7.015f;
    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] = (xv[2] - xv[0])
        + (-0.7169861741f * yv[0]) + (1.4453653501f * yv[1]);
    return yv[2];
}

uint8_t beat_judge(float val){
  static float history[10];
  int i = 0;
  float avg =0;

  //compute average of last 9 samples (hopefully)
  for(i = 9; i >= 0; i--){
    avg += history[i];
  }
  avg = avg/9;


  //write history (heh, see what I did there? no? nevermind. Just pushing newest value on FIFO)
  for(i = 0; i< 8; i++){
    history[i] = history[i+1];
  }
  history[9] = val;

  //debug stuff
  //Serial.print("Avg:");
  //Serial.println(avg);
  //Serial.println(val);


  //decide
  if((avg * 145) < (val-45)){ //"magic" (adapt this garbage to something that works better, if possible)
  //if((avg * 100) < (val-45)){ //"magic" (adapt this garbage to something that works better, if possible)
    //basically we got fast rise in low freq volume
    return 1;
  }else{
    //fake news
    return 0;
  }
  
}

#define MAX_X_VAL 222
void scope_timer_callback(void * param)
{
  static uint8_t samples = 0;
  float adc_val_for_beat,value, envelope, beat = 0;
  uint8_t judge_result = 0;
  static uint8_t beat_state = 0; //0 for off, 1 for beam off, 2 for beat on, 3 for beam off
  static volatile uint32_t time_in_state = 0;
  static volatile uint8_t offset_temp = 0;
  static volatile uint8_t offset_start = 0;
  static volatile uint8_t offset_stop = MAX_X_VAL;

  int adc_val;
  int filtered_val ;
  static int32_t adc_val_old = 4096/2;
  
  static uint8_t x;
  static uint8_t direction = 1;
  uint8_t y;

  //get data
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_val));

  // compute data

  //beat  detect
  adc_val_for_beat = adc_val - 1430; //subtract offset

  // Filter only bass component
  value = bassFilter(adc_val_for_beat);

  // Take signal amplitude and filter (basically get the envelope of the low freq signals)
  if(value < 0) value=-value;
  envelope = envelopeFilter(value);

  if(samples == 200){
    samples = 0;

    // Filter out repeating bass sounds 100 - 180bpm
    beat = beatFilter(envelope);
    judge_result = beat_judge(beat);
    //if(beat > 2){
      //printf(">\n");
      //offset_temp = 100;
    //}
  }else{
    samples++;
  }

  //small state machine so we can control reaction time to beat
  switch(beat_state){
    case 0://no beat
      if(time_in_state){//we still have to wait in this state
        time_in_state--;
      }else{
        if(judge_result){//if beat should be considered as valid
          time_in_state = 3;//set time spent in beam off
          gpio_set_level(GPIO_NUM_27, 0); // actual beam off
          beat_state = 1;//goto beam off
        }
      }
      break;
    case 1://beam off
      offset_temp = (esp_random()%50)-25;
      offset_start = (esp_random()%100);
      offset_stop = MAX_X_VAL - (esp_random()%100);
      if(time_in_state){//we still have to wait in this state
        time_in_state--;
      }else{
          time_in_state = 100;//set time spent in "Active" beat
          gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
          beat_state = 2;//go to beat "active"
      }
      break;
    case 2: //beat "active"
      if(time_in_state){
        time_in_state--;
      }else{
        time_in_state = 3;//set minimum time spent in beam off state
        gpio_set_level(GPIO_NUM_27, 0); // actual beam off
        beat_state = 3;
      }
      break;
    case 3://beam off
      //offset_temp = 0;
      if(time_in_state){//we still have to wait in this state
        time_in_state--;
      }else{
          time_in_state = 2000;//set time spent in "no" beat
          gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);//switch beam on
          beat_state = 0;//go to beat "off"
      }
      break;
  }

//#define FILT_TIME2  85
//#define FILT_TIME2  165 //best so far
#define FILT_TIME2  130
  // limit values
  if (adc_val < (FILT_TIME2 + 1))
    {adc_val = FILT_TIME2;}
  if (adc_val > (4090 - FILT_TIME2))
    {adc_val = (4090 - FILT_TIME2);}

    if (adc_val > (adc_val_old + FILT_TIME2)) {
       filtered_val = adc_val_old + FILT_TIME2;
    } else if (adc_val < (adc_val_old - FILT_TIME2)) {
       filtered_val = adc_val_old - FILT_TIME2;
    } else {
       filtered_val = adc_val;
    }

    adc_val_old = filtered_val;

  //advance point
    if (direction) {
       x++;
       if (x == 222) {
       direction = 0;
       }
    } else {
       x--;
       if (x == 0) {
       direction = 1;
       }
    }

    //y = filtered_val / 20; //was ok before gain increase
    y = filtered_val / 50;

  
  if(x < offset_start){
    gpio_set_level(GPIO_NUM_27, 0);
  }else{
    gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
  }

  if(x > offset_stop){
    gpio_set_level(GPIO_NUM_27, 0);
  }

    //"print" data
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan0_handle, x));
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan1_handle, y+offset_temp));
}

static void dac_output_task(void *args)
{
    dac_oneshot_handle_t handle = (dac_oneshot_handle_t)args;
    while (1) {
        /* Set the voltage every 100 ms */
        ESP_ERROR_CHECK(dac_oneshot_output_voltage(handle, val));
        val += 10;
        val %= 250;
        //vTaskDelay(pdMS_TO_TICKS(10));
        vTaskDelay( 8/ portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan0_cfg, &chan0_handle));
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &chan1_handle));

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));
    //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //gpio_pad_select_gpio(GPIO_NUM_27);
    //gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_27 | 1ULL<<GPIO_NUM_5 | 1ULL<<GPIO_NUM_18 | 1ULL<<GPIO_NUM_19);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_17);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

  gpio_set_level(GPIO_NUM_27, GPIO_NUM_27);
  gpio_set_level(GPIO_NUM_5, 0);
  gpio_set_level(GPIO_NUM_18, 0);
  esp_timer_create_args_t my_timer_args = {
      //.callback = &timer_callback,
      .callback = &scope_timer_callback,
      .name = "My Timer"};
  esp_timer_handle_t timer_handler;
  ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
  //ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 55)); //for tree
  //ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 385)); //first scope
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 200)); //best for second scope
  //ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 180));//much better for second scope
  

    /* DAC oneshot outputting threads */
   // xTaskCreate(dac_output_task, "dac_chan0_output_task", 4096, chan0_handle, 5, NULL);
   // vTaskDelay(pdMS_TO_TICKS(500)); // To differential the output of two channels
   // xTaskCreate(dac_output_task, "dac_chan1_output_task", 4096, chan1_handle, 5, NULL);


  uint8_t current_anim = 0;
  uint8_t led_state = 0;

  while(1)
  {
    //int32_t adc_val = 0;
    //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_val));
    //printf("%ld\n", adc_val);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    if(!gpio_get_level(GPIO_NUM_17)/*button pushed*/)//search for short between gpio 5 and 17
    {
      switch(current_anim){ 
        case 0: //if in anim 0 (tree), setup and go to anim 1 (scope)
          //setup scope
          ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
          ESP_ERROR_CHECK(esp_timer_delete(timer_handler));
          my_timer_args.callback = &scope_timer_callback;
          ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
          ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 200)); //best for second scope
          //switch state to scope
          current_anim = 1;
          printf("Going to scope\n");
          break;
        case 1: //if in anim 1 (scope), setup and go to anim 0 (tree)
          //setup tree
          ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
          ESP_ERROR_CHECK(esp_timer_delete(timer_handler));
          my_timer_args.callback = &timer_callback;
          ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
          ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 55)); //for tree
          //switch state to tree
          current_anim = 0;
          printf("Going to tree\n");
          break;
      }
    }

    if(current_anim == 1){ //if in scope, blink led
      if(led_state){
        gpio_set_level(GPIO_NUM_19, 0);
        led_state = 0;
      }else{
        gpio_set_level(GPIO_NUM_19, GPIO_NUM_19);
        led_state = 1;
      }
    }else{//if not in scope, turn led on
      gpio_set_level(GPIO_NUM_19, GPIO_NUM_19);
    }
  }

}
