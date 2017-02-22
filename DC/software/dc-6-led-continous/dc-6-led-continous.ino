#include <limits.h>


#define SENSOR_PIN_0 A0
#define SENSOR_PIN_1 A1

#define NUM_OF_TX 6

uint16_t *measured_values;

uint8_t leds[] = {8, 9, 10, 11, 12, 13};

uint8_t start_state_per_led[] = {0, 1, 2, 3, 4, 5, 6};

uint16_t offset_per_led[] = {32, 6, 9, 64, 7, 10};

uint8_t **tx_code;

#define ADC_BUFFER_SIZE 2500
uint16_t *adc_buffer = new uint16_t[ADC_BUFFER_SIZE];

uint16_t adc_idx;

uint8_t adc_buffer_full = 0;




//uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1
//uint8_t poly2[] = {1, 1, 1, 1, 0, 1}; // x^5 + x^4 + x^3 +x^2 + 1

//uint8_t poly[] = {1, 0, 0, 0, 0, 1, 1}; // x^6 + x + 1
//uint8_t poly2[] = {1, 1, 0, 0, 1, 1, 1}; // x^6 + x^5 + x^2 + x + 1

uint8_t poly[] = {1, 0, 0, 0, 1, 0, 0, 1}; // x^7 + x^3 + 1
uint8_t poly2[] = {1, 0, 0, 0, 1, 1, 1, 1}; // x^7 + x^3 + x^2 + x + 1

//uint8_t poly[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 1}; // x^9 + x^4 + 1
//uint8_t poly2[] = {1, 0, 0, 1, 0, 1, 1, 0, 0, 1}; // x^9 + x^6 + x^4 + x^3 + 1

//uint8_t poly[] = {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1}; // x^10 + x^2 + 1
//uint8_t poly2[] = {1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1}; // x^10 + x^8 + x^3 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint16_t num_of_balanced_gold_seq;
uint16_t *balanced_gold_seq_start_states = new uint16_t[N];



/*
This function takes two polynomials arrays and the shift register length n, 
writes the number of balanced gold seqs it produces 
and writes to an array the start states of the second register to obtain balanced gold codes.\

call like so :

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p1[] = {...};
uint8_t p2[] = {...};

uint8_t Nb;

uint8_t bgs = new uint8_t[L];

calc_balanced_gold_codes_idx(p1, p2, n, Nb, bgs);
*/
void calc_balanced_gold_codes_idx(uint8_t *pref_poly1, uint8_t *pref_poly2, uint8_t n, uint16_t &num_of_balanced_gold_seq, uint16_t *balanced_gold_seq_start_states) {
  uint16_t L = (1 << n) - 1;
  uint16_t N = (1 << n) + 1;

  uint16_t start_state1 = 1;

  num_of_balanced_gold_seq = 0;

  for (uint16_t start_state2 = 1; start_state2 <= L; start_state2++) {

    uint16_t lfsr1 = start_state1;
    uint16_t lfsr2 = start_state2;

    uint8_t out1;
    uint8_t out2;

    uint8_t gold_out;
    uint16_t sum_of_gold_seq = 0;

    do {
      out1 = out2 = 0;

      for (uint8_t i = n; i > 0; i--) {
        if (pref_poly1[i] == 1) {
          out1 ^= (lfsr1 >> (n - i));
        }

        if (pref_poly2[i] == 1) {
          out2 ^= (lfsr2 >> (n - i));
        }
      }

      out1 &= 1;
      out2 &= 1;

      gold_out = out1 ^ out2;
      sum_of_gold_seq += gold_out;

      lfsr1 = (lfsr1 >> 1) | (out1 << (n - 1));
      lfsr2 = (lfsr2 >> 1) | (out2 << (n - 1));

    } while (lfsr1 != start_state1);


    if (sum_of_gold_seq == (1 << (n - 1))) {
      // Half + 1 are ones, so balanced
      // save the start_state2;
      balanced_gold_seq_start_states[num_of_balanced_gold_seq++] = start_state2;
    }


  }

}

/*
This function takes two polynomials arrays and the shift register length n, 
and the start state for the second LFSR
and writes to an other array the gold code.

call like so:

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p1[] = {...};
uint8_t p2[] = {...};

uint8_t *gs = new uint8_t[L];


gold_seq_create(p1, p2, n, 1, gs);

*/
void gold_seq_create(uint8_t *pref_poly1, uint8_t *pref_poly2, uint8_t n, uint16_t start_state2, uint8_t *gold_seq) {
  //uint16_t L = (1 << n) - 1;
  //gold_seq = new uint8_t[L];
  
  uint16_t start_state1 = 1;

  uint16_t lfsr1 = start_state1;
  uint16_t lfsr2 = start_state2;

  uint8_t out1;
  uint8_t out2;

  uint16_t step_pos = 0;
  
  do {
    out1 = out2 = 0;

    for (uint8_t i = n; i > 0; i--) {
      if (pref_poly1[i] == 1) {
        out1 ^= (lfsr1 >> (n - i));
      }

      if (pref_poly2[i] == 1) {
        out2 ^= (lfsr2 >> (n - i));
      }
    }

    out1 &= 1;
    out2 &= 1;

    gold_seq[step_pos++] = out1 ^ out2;

    lfsr1 = (lfsr1 >> 1) | (out1 << (n - 1));
    lfsr2 = (lfsr2 >> 1) | (out2 << (n - 1));

  } while (lfsr1 != start_state1);
}


/*
This function creates an m-sequence with a given polynomial.

call like so:

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p[] = {...};

uint8_t *ms = new uint8_t[L];

m_seq_create(p, n, 1, ms);


*/
void m_seq_create(uint8_t *poly, uint8_t n, uint16_t start_state, uint8_t *m_seq) {
  uint16_t L = (1 << n) - 1;
    
  uint16_t lfsr = start_state;
  uint16_t bit_out;
  uint16_t step_pos = 0;

  
  do {
    bit_out = 0;
    for (uint8_t i = n; i > 0; i--)
      if (poly[i] == 1)
        bit_out ^= (lfsr >> (n-i));
    bit_out &= 1;
    
    lfsr = (lfsr >> 1) | (bit_out << (n-1));
    
    m_seq[step_pos++] = bit_out;
  } while (lfsr != start_state);
  
}





int16_t find_min(int16_t *array, uint16_t length) {
  int16_t min = SHRT_MAX;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] < min)
      min = array[i];
  }

  return min;
}

int16_t find_max(int16_t *array, uint16_t length) {
  int16_t max = SHRT_MIN;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] > max)
      max = array[i];
  }

  return max;
}





 


void setup() {

  Serial.begin(115200); 

  adc_idx = 0;

  
  calc_balanced_gold_codes_idx(poly, poly2, n, num_of_balanced_gold_seq, balanced_gold_seq_start_states);

  tx_code = new uint8_t*[NUM_OF_TX + 1];


  for (uint8_t i = 0; i < (NUM_OF_TX + 1); i++) {

    tx_code[i] = new uint8_t[L];

    
    gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[start_state_per_led[i]], tx_code[i]);


    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  measured_values = new uint16_t[L];

  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    digitalWrite(leds[i], LOW);
  }





  /*for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    Serial.print(i);Serial.print(": ");
    for (uint16_t j = 0; j < L; j++) {
      Serial.print(tx_code[i][j]); Serial.print(" ");
    }
    Serial.println();
  }*/

}



void loop() {

  for (uint16_t chip = 0; chip < L; chip++) {

    
    for (uint8_t led = 0; led < NUM_OF_TX; led++){
      digitalWrite(leds[led], tx_code[led][(chip + offset_per_led[led]) % L]);
    }

    delayMicroseconds(100);

    adc_buffer[adc_idx++] = analogRead(SENSOR_PIN_0) + analogRead(SENSOR_PIN_1);

    if (adc_idx > ADC_BUFFER_SIZE) {
      adc_buffer_full = 1;
      adc_idx = 0;
      break;
    }
      

  }


#define SLOPE_NOISE 50

  if (adc_buffer_full) {

    Serial.println("RAW ADC: ");
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }
    Serial.println("************************************");

    Serial.println("Correlation");

    for (uint16_t offset = 1; offset < (ADC_BUFFER_SIZE - L); offset++) {

      uint16_t abs_max_slope = 0;
      
      uint16_t abs_slope_per_led = 4095; // smallest absolute slope, but bigger than (around) 0

      int16_t *diff_signal = new int16_t[L - 1];

      uint16_t min_signal = 4095;
      
      for (uint16_t i = 0; i < (L - 1); i++) {
        diff_signal[i] = ((int16_t)adc_buffer[i + offset]) - ((int16_t)adc_buffer[i + offset - 1]);

        uint16_t abs_slope = abs( diff_signal[i] );
        
        if (abs_slope > SLOPE_NOISE) {
          abs_slope_per_led = min(abs_slope_per_led, abs_slope);
        }
        
        abs_max_slope = max(abs_max_slope, abs_slope);

        min_signal = min(min_signal, adc_buffer[i + offset - 1]);
      }

      if (abs_max_slope <= SLOPE_NOISE) {
        // not so much change in signal, so probably no LEDs modulating, add 1 to number times off

        
      } else {

        int16_t *improved_signal = new int16_t[L];

        uint32_t signal_sum = 0;
        
        for (uint16_t i = 0; i < L; i++) {
          // post process signal, remove constants and noise at bottom end.
          improved_signal[i] = adc_buffer[i + offset] - min_signal;
          if (improved_signal[i] <= SLOPE_NOISE)
            improved_signal[i] = 0;
          
          signal_sum += improved_signal[i];
        }

        float calc_num_of_tx = (float)signal_sum / (abs_slope_per_led * (1 << (n - 1)));
      
        if ( calc_num_of_tx < 10 /*(1 << (n + µ1))*/) {
          // acceptable number...


          for (uint8_t led = 6; led < 7/*NUM_OF_LEDS*/; led++) {
            int32_t corr_sum = 0;

            for (uint16_t i = 0; i < L; i++) {
              int8_t r_chip = 1 - 2 * ((int8_t)tx_code[led][i]); // for optimizing maybe do this already at code creation......
              corr_sum += (int16_t)improved_signal[i] * r_chip;
            }

            float normalized_l_corr = ((float)-2 * corr_sum / abs_slope_per_led) - calc_num_of_tx;

            //check if correlation is somewhat in bounds..
            //if ( (normalized_l_corr < (1.4*L)) &&  (normalized_l_corr > (-1.4*L)) ) {
              
              /*uint8_t state_of_led = normalized_l_corr > ((float)L/2.0);

              Serial.println(state_of_led);*/

              Serial.println(normalized_l_corr);
              
            /*} else {
              // not within reasonable bounds
            }*/
            
          }
          
        } else {
          
          // not reasonable number,
          
        }

        delete improved_signal;
  
      }

      delete diff_signal;

      
      

      

      
      
    }

    
    Serial.println("----------------------------------------");

    adc_buffer_full = 0;
  }











}







