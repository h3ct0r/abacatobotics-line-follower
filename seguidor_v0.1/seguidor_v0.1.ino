#include <QTRSensors.h>
#include "RunningAverage.h"

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) {
    36, 34, 32, 30, 28, 26, 24, 22
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int led_left = 38;
int led_right = 40;

// H-bridge pins (Defines the velocity)
int pwm_a = 10;   //PWM motor 1 
int pwm_b = 5;    //PWM motor 2
int VEL = 100; // Constante que atualizará a velocidade dos motores
int in1 = 9;
int in2 = 8;
int in3 = 7;
int in4 = 6;

int mode = 0;

int last_proportional;
int integral;

// Control
float error = 0;
float last_error = 0;
float PV = 0;
float kp = 0;
float kd = 0;
int m1_speed = 0;
int m2_speed = 0;

// Running average of the last measurements
RunningAverage average_position(5);
int samples = 0;

int alert_obstacle = 0;

#define ROW 6
#define COLUMN 10

#define BLACK_SQUARE_MIDDLE     0
#define BLACK_EMPTY_SPACE       1
#define WHITE_SQUARE_RIGHT_0    2
#define WHITE_SQUARE_RIGHT_1    3
#define WHITE_SQUARE_LEFT_0     4
#define WHITE_SQUARE_LEFT_1     5

int obstacle_matrix [ROW][COLUMN] =
  {
    {   0,    0,    0,    0, 1000, 1000,    0,    0,    0,    0},     //black_square_in_middle
    {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000},     //black_empty_space
    {1000, 1000, 1000,  500,    0,    0,  500, 1000, 1000,    0},     //white_square_right
    {1000, 1000, 1000,  500,    0,    0,  500, 1000,    0,    0},     //white_square_right
    {   0, 1000, 1000,  500,    0,    0,  500, 1000, 1000, 1000},     //white_square_left
    {   0,    0, 1000,  500,    0,    0,  500, 1000, 1000, 1000}      //white_square_left
};

#define STATE_FOLLOW_LINE                   0
#define STATE_STOP_10                       1
#define STATE_CURVE_RIGHT                   2
#define STATE_CURVE_LEFT                    3
#define STATE_FORWARD_UNTIL_LINE_IS_FOUND   4

int current_state = 0;
int obstacle_loop_counter = 0;
int obstacle_detected = -1;

void setup()
{
    average_position.clear(); // explicitly start clean
    
    //Set control pins to be outputs
    pinMode(pwm_a, OUTPUT);
    pinMode(pwm_b, OUTPUT);
    
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    pinMode(led_left, INPUT);
    pinMode(led_right, INPUT);
    
    Serial.begin(9600);
    
    Serial.println("Calibration started...");
    
    // calibrate line sensor
    delay(500);
    pinMode(13, OUTPUT);
    
    digitalWrite(13, HIGH);         // turn on Arduino's LED to indicate we are in calibration mode
    int change_led_status = 6;
    int change_dir = 50;
    
    int cicle_count = 0;
    
    for (int i = 0; i < 300; i++)   // make the calibration take about 10 seconds
    {
        if (cicle_count >= change_led_status) {
            digitalWrite(13, !digitalRead(13));
            cicle_count = 0;
        }
        
        qtrrc.calibrate();
        cicle_count++;
        delay(10);
    }
    digitalWrite(13, LOW);          // turn off Arduino's LED to indicate we are through with calibration
    
    // print the calibration minimum values measured when emitters were on
    /*
    Serial.println("nSensor min values:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(qtrrc.calibratedMinimumOn[i]);
        Serial.print(' ');
    }
    Serial.println();
    
    // print the calibration maximum values measured when emitters were on
    Serial.println("nSensor max values:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(qtrrc.calibratedMaximumOn[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
    Serial.println("Calibration Complete");
    delay(1000);
    
}

void loop()
{
    unsigned int line_position = qtrrc.readLine(sensorValues);

    // ------------------------------------- //
    //          Obstacle detection           //
    // ------------------------------------- //

    // Check if there are obstacles every 2 cicles
    if(obstacle_loop_counter > 1){
        obstacle_detected = detect_obstacle();
        obstacle_loop_counter = 0;
    }
    else{
        obstacle_detected = -1;
        obstacle_loop_counter++;
    }

    Serial.print("Obstacle detection: ");
    Serial.println(obstacle_detected);
    Serial.println();
    Serial.println();
    
    if(obstacle_detected == BLACK_SQUARE_MIDDLE){
        current_state = STATE_FOLLOW_LINE;
        alert_obstacle = 80;
    }
    else if(obstacle_detected == BLACK_EMPTY_SPACE){
        if(current_state == STATE_FOLLOW_LINE){
            current_state = STATE_STOP_10;
        }
    }
    else if(obstacle_detected == WHITE_SQUARE_RIGHT_0 || obstacle_detected == WHITE_SQUARE_RIGHT_1){
        if(current_state == STATE_FOLLOW_LINE){
            current_state = STATE_CURVE_RIGHT;
        }
    }
    else if(obstacle_detected == WHITE_SQUARE_LEFT_0 || obstacle_detected == WHITE_SQUARE_LEFT_1){
        if(current_state == STATE_FOLLOW_LINE){
            current_state = STATE_CURVE_LEFT;
        }
    }

    // ------------------------------------- //
    //             State machine             //
    // ------------------------------------- //
    
    if(current_state == STATE_STOP_10){
        // Stops approx 10 seconds and then go forward
        
        analogWrite(pwm_a, 10);
        analogWrite(pwm_b, 10);
        delay(100); 
        
        for (int i = 0; i < 400; i++){
            analogWrite(pwm_a, 0);
            analogWrite(pwm_b, 0);
            delay(10); 
        }
  
        current_state = STATE_FORWARD_UNTIL_LINE_IS_FOUND;
    }
    else if(current_state == STATE_FORWARD_UNTIL_LINE_IS_FOUND){
        analogWrite(pwm_a, 100);
        analogWrite(pwm_b, 100);
        if(line_position > 0 && line_position < 7000){
            current_state = STATE_FOLLOW_LINE;
        }
    }
    else if(current_state == STATE_CURVE_RIGHT){
        //todo: go to right until find line
        Serial.println("State CURVE RIGHT");
        current_state = STATE_FOLLOW_LINE;
    }
    else if(current_state == STATE_CURVE_LEFT){
        //todo: go to left until find line
        Serial.println("State CURVE LEFT");
        current_state = STATE_FOLLOW_LINE;
    }
    else{
        //todo: Make the car go slower when alert_obstacle its greater than 0
        if(alert_obstacle > 0){
            follow_line(line_position);
        }
        else{
            follow_line(line_position);
        } 
    }

    Serial.print("Current State: ");
    Serial.println(current_state);
    
    delay(10);
    
    if(alert_obstacle > 0){
        --alert_obstacle;
    }
}



int detect_obstacle(){
    // Initialize the vector with 100% difference
    int result[ROW];
    for(int i = 0; i < ROW; i++){
        result[i] = 100;
    }

    int sensorValuesWithLateral[COLUMN];
    int i2 = 7;
    for(int i = 1; i < COLUMN - 1; i++){
        sensorValuesWithLateral[i] = sensorValues[i2];
        i2--;
    }
  
    sensorValuesWithLateral[0] = digitalRead(led_left) * 1000;
    sensorValuesWithLateral[COLUMN - 1] = digitalRead(led_right) * 1000;

    long avg = 0;

    for (int i = 0; i < ROW; i++){
        avg = 0;
        for (int j = 0; j < COLUMN; j++){
            int v1 = obstacle_matrix[i][j] - sensorValuesWithLateral[j];
            int abs_v1 = abs(v1);
            avg += (abs_v1 / 10);
        }
        result[i] = (avg / COLUMN);
    }

    /*
    Serial.println("Sensor values:");
    for(int i = 0; i < COLUMN; i++){
        Serial.print(sensorValuesWithLateral[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.println("Detected values:");
    for(int i = 0; i < ROW; i++){
        Serial.print(i);
        Serial.print(") ");
        Serial.print(result[i]);
        Serial.print(", ");
    }
    Serial.println();
    */

    int treshold = 15; // 100 - 15 = 75% of similarity
    int best_val = 9999;
    int best_pos = -1;
    for(int i = 0; i < ROW; i++){
        if(result[i] < best_val && result[i] < treshold){
            best_pos = i;
            best_val = result[i];
        }
    }

    return best_pos;
}

int detect_black_obstacle(){
    int sum = 0;
    int all_sensors_high = 1;
    int tresh_min = 500;
    int tresh_max = 1900;
    
    for (int i = 0; i < NUM_SENSORS; i++){
        int value = sensorValues[i];
        
        if(value < 950){
            all_sensors_high = 0;
        }
        
        if(value > 50) {
            sum += value;
        }
    }

    if(all_sensors_high) return 0;

    if(sum >= tresh_min && sum <= tresh_max){
        return 1;
    }
    else{
        return 0;  
    }
}

void follow_line(int line_position) {
    switch (line_position) {
        // Line has moved off the left edge of sensor
        case 0:
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
       
            analogWrite(pwm_a, 0);
            analogWrite(pwm_b, 0);
            Serial.println("Rotate Left/n");
        break;
        
        // Line had moved off the right edge of sensor
        case 7000:
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
        
            analogWrite(pwm_a, 0);
            analogWrite(pwm_b, 0);
            Serial.println("Rotate Right/n");
        break;

        default:
            // Obtém a posição da linha
            // Aqui não estamos interessados nos valores individuais de cada sensor

            // O termo proporcional deve ser 0 quando estamos na linha
            int proportional = ((int)line_position) - 3500;
    
            // Calcula o termo derivativo (mudança) e o termo integral (soma)
            // da posição
            int derivative = proportional - last_proportional;
            integral += proportional;
            
            // Lembrando a ultima posição
            last_proportional = proportional;
            
            // Calcula a diferença entre o aranjo de potência dos dois motores
            // m1 - m2. Se for um número positivo, o robot irá virar para a 
            // direita. Se for um número negativo, o robot irá virar para a esquerda
            // e a magnetude dos números determinam a agudez com que fará as curvas/giros
            PV = proportional/3 + integral/10000 + derivative*12/2;
      
            //this codes limits the PV (motor speed pwm value)
            if (PV > VEL){
                PV = VEL;
            }
      
            if (PV < -VEL){
                PV = -VEL;
            }
      
            m1_speed = VEL+PV;
            m2_speed = VEL-PV;
      
            //set motor speeds and direction
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH); 
            analogWrite(pwm_a, m1_speed);
            analogWrite(pwm_b, m2_speed);
        break;
    }
}
