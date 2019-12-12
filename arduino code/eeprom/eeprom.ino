#include <EEPROM.h>

int prom_cursor = 0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void write_prom(float t, float h, int current_time){

  if (!isnan(h) || !isnan(t)){

    if(current_time/freq > write_count-1){

      //Serial.println("writing");

      // minus one is there to caputre the very first humidity reading

      int modified_t = 255.0*(t - min_temp)/(max_temp-min_temp);
      int modified_RH = 255.0*(h - min_RH)/(max_RH - min_RH);

      EEPROM.write(prom_cursor, modified_t);
      prom_cursor = prom_cursor + 1;
      EEPROM.write(prom_cursor, modified_RH);
      prom_cursor = prom_cursor + 1;

      write_count = write_count + 1;
    
    }
    
  }
  
}

void erase_prom(){

  for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
   }

   prom_cursor = 0;

}
