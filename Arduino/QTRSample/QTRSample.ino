//volatile long enc_count = 0;
//
////8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
// 
//void setup() {
//    Serial.begin(9600);
//
//    // all your normal setup code
//    attachInterrupt(0,encoder_isr,CHANGE);
//    attachInterrupt(1,encoder_isr,CHANGE);
//
//
//}
//
//void loop(){
//  
//}
//
//void encoder_isr() {
//    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
//    static uint8_t enc_val = 0;
//    
//    enc_val = enc_val << 2;
//    enc_val = enc_val | ((PIND & 0b1100) >> 2);
// 
//    enc_count = enc_count + lookup_table[enc_val & 0b1111];
//
//    Serial.print("enc_count = ");
//    Serial.println(enc_count);
//}

volatile long enc_count_right = 0;
volatile long enc_count_left = 0;
int8_t lookup_table[] = {1,-1,-1,1};


//8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
 
void setup() {
    Serial.begin(9600);

    // all your normal setup code
    attachInterrupt(0,encoder_isr_right,CHANGE); // right pin 2
    attachInterrupt(1,encoder_isr_left,CHANGE); // left pin 3


}

void loop(){
  
}

void encoder_isr_right() {

    byte current = (PIND & 0b01000100) >> 2;

    if (current > 2) {
      current &= 0b01;
      current |= 0b10;
    }

    enc_count_right = enc_count_right + lookup_table[current];

    Serial.print("right enc_count = ");
    Serial.println(enc_count_right);
}

void encoder_isr_left() {
   
    byte current = (PIND & 0b10001000) >> 3;

    if (current > 2) {
       current -= 0b10000;
      current += 0b00010;
    }

    enc_count_left = enc_count_left + lookup_table[current];
    Serial.print("left enc_count = ");
    Serial.println(enc_count_left);
}
