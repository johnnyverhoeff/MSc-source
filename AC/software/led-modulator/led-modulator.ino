
#define START_STATE 0
//#define START_STATE 1
//#define START_STATE 2

#define CONT_TX 
// define this for continous modulation, 
//undefine this to modulate for every incoming Serial character

#define NUM_OF_MODULATION_SEQUENCES 1
// only actually relevant for serial transmission, not for cont_tx

#define NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL 100
// for auto-sensing trigger signal length

//#define HARDCODE_MODULATE_TIME
// define this for hardcoded 7 ms, else for auto-detection

#define MODULATE_ENABLE_TRIGGER_STATE 1
// if 1, modulation is allowed at trigger input HIGH
// else if 0, modulation is allowed at trigger input LOW
// 0 for 'old' TIP50 current source (working example).
// 1 for 'new' BUZ80 current source (WIP).

#define LED_ON 0
#define LED_OFF !LED_ON
// defines if the LED circuit is positive-logic or negative-logic
// LED_ON 1, for 'old' TIP50
// LED_ON 0, for 'new' BUZ80


#define modulate_enable 3
#define led 13

uint32_t  time_to_modulate_per_period,
          modulate_times_per_period;

const uint32_t timer_freq = 10000; //Hz
          

volatile uint16_t timer1_counter;


volatile uint32_t loop_counter = 0;


volatile uint8_t modulate_idx = 0;


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

uint8_t *m_seq;
volatile uint16_t m_seq_idx;

volatile uint8_t timer_enable = 0;

volatile uint8_t done_modulating_one_seq = 0;

volatile uint8_t modulate_enable_flag = 0;





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






uint32_t get_avg_modulate_enable_time(void) {
  uint32_t  avg_modulate_enable_time = 0, 
          min_modulate_enable_time = 10000,
          max_modulate_enable_time = 0,
          
          avg_modulate_disable_time = 0,
          min_modulate_disable_time = 10000,
          max_modulate_disable_time = 0;
          
  for (int i = 0; i < NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL; i++) {
    
    while (digitalRead(modulate_enable) == MODULATE_ENABLE_TRIGGER_STATE); //wait while signal is low
    while (digitalRead(modulate_enable) == !MODULATE_ENABLE_TRIGGER_STATE); //wait while signal is high
    
    //signal is low.
    
    uint32_t begin_time_modulate_enable = micros();
  
    while (digitalRead(modulate_enable) == MODULATE_ENABLE_TRIGGER_STATE); //wait while signal is low
  
    uint32_t end_time_modulate_enable = micros();
    uint32_t begin_time_modulate_disable = end_time_modulate_enable;
  
    while (digitalRead(modulate_enable) == !MODULATE_ENABLE_TRIGGER_STATE); //wait while signal is high
  
    uint32_t end_time_modulate_disable = micros();

    uint32_t modulate_enable_time = end_time_modulate_enable - begin_time_modulate_enable;
    uint32_t modulate_disable_time = end_time_modulate_disable - begin_time_modulate_disable;

    min_modulate_enable_time = min(min_modulate_enable_time, modulate_enable_time);
    max_modulate_enable_time = max(max_modulate_enable_time, modulate_enable_time);

    min_modulate_disable_time = min(min_modulate_disable_time, modulate_disable_time);
    max_modulate_disable_time = max(max_modulate_disable_time, modulate_disable_time);

    avg_modulate_enable_time = (avg_modulate_enable_time + (min_modulate_enable_time + max_modulate_enable_time) / 2) / 2;
    avg_modulate_disable_time = (avg_modulate_disable_time + (min_modulate_disable_time + max_modulate_disable_time) / 2) / 2;

  }

  uint32_t avg_period_time = avg_modulate_enable_time + avg_modulate_disable_time;

  if (avg_period_time > 10100) {
    Serial.print("Sanity check failed...");
    Serial.println(avg_period_time);
    return get_avg_modulate_enable_time();
  }

  return avg_modulate_enable_time;
}

void enable_timer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer() {
  TIMSK1 &= ~(1 << TOIE1); 
}

void init_timer() {
  // initialize timer1 -
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter = 65536 - (16 * 1000000 / 256 / timer_freq);

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
}


void setup() {
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);

  pinMode(led, OUTPUT);
  digitalWrite(led, LED_OFF);

#ifdef HARDCODE_MODULATE_TIME

  time_to_modulate_per_period = 7000; //us
  
#else 

  uint32_t avg_modulate_enable_time = get_avg_modulate_enable_time();

  time_to_modulate_per_period = min(7000 /* us */, avg_modulate_enable_time);

  Serial.print("avg_modulate_enable_time: "); Serial.println(avg_modulate_enable_time);
  
#endif

  
  modulate_times_per_period = time_to_modulate_per_period * timer_freq / 1000000 - 1;

  Serial.print("time_to_modulate_per_period: "); Serial.println(time_to_modulate_per_period);
  Serial.print("modulate_times_per_period: "); Serial.println(modulate_times_per_period);

  
  digitalWrite(led, LED_ON);

  m_seq = new uint8_t[L];
  m_seq_idx = 0;


  calc_balanced_gold_codes_idx(poly, poly2, n, num_of_balanced_gold_seq, balanced_gold_seq_start_states);

  gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[START_STATE], m_seq);

  noInterrupts();
  cli();

  modulate_idx = 0;

  init_timer();
  disable_timer();
     
  modulate_enable_flag = 0;
  
  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );

  interrupts();       
  sei();
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer
  
  if (modulate_idx < modulate_times_per_period) {

    digitalWrite(led, !(m_seq[m_seq_idx] ^ LED_ON));

    m_seq_idx++;
    modulate_idx++;

    if (m_seq_idx >= L) {
      m_seq_idx = 0;
      done_modulating_one_seq++;

      if (done_modulating_one_seq >= NUM_OF_MODULATION_SEQUENCES) {
        modulate_enable_flag = 0;
        
        done_modulating_one_seq = 0;
        //Serial.println("DONE");
      }
    }
  } else {
    digitalWrite(led, LED_ON);
    disable_timer();
  }
}


void isr_change(void) {
  if (modulate_enable_flag == 1) {

    int state = digitalRead(modulate_enable);
  
    if (state == MODULATE_ENABLE_TRIGGER_STATE) {
      
      enable_timer();
    
    } else {
      
      modulate_idx = 0;
      disable_timer();
  
      digitalWrite(led, LED_ON);
      
    } 
    
  } else {
    digitalWrite(led, LED_ON);
  }
} 


void loop() {  

  // to make sure the timer wont fail...
  loop_counter++;
  if (loop_counter % 10000 == 0) {
    init_timer();
  }

#ifdef CONT_TX
  
  modulate_enable_flag = 1;

#else 

  if (modulate_enable_flag == 0) {
    if (Serial.available() > 0) {
      Serial.print("Received: "); Serial.println(Serial.read());
     
      while (Serial.available() > 0) {
        Serial.println(Serial.read());
      }
 
      modulate_enable_flag = 1;
    }
  }

#endif

}


