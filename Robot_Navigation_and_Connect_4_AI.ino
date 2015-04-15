#include <math.h>
#include <Servo.h>
Servo myServoClaw;
Servo myServoGate;
Servo myDisposalServo;

const int disposalEnablePin = 11;
const int fanEnablePin = 9;                                             
const int clawServoPin = 10;                                            // Note: As angle increases, claw opens more
const int gateServoPin = 12;                                            // Note: As angle increases, gate closes more

const int enablePin = 4;
const int leftPin = 6;
const int rightPin = 5;

const int trigPinTop = 40;
const int echoPinTop = 41;
const int trigPinBottom = 42;
const int echoPinBottom = 43;
const int trigPinLeft = 44;
const int echoPinLeft = 45;
const int trigPinRight = 46;
const int echoPinRight = 47;

const int column1 = 48;
const int column2 = 49;
const int column3 = 50;
const int column4 = 51;
const int column5 = 52;
int columns[5] = {column1, column2, column3, column4, column5};

const int row1 = 22;
const int row2 = 23;
const int row3 = 24;
const int row4 = 25;
const int row5 = 26;
const int row6 = 27;
const int row7 = 28;
int rows[7] = {row1, row2, row3, row4, row5, row6, row7};

const float pi = 3.141592653589;

float TargetX;
float TargetY;
float xposition;
float yposition;
float return_position[2] = {-1, -1};

// Keep track of how many balls are in the hoppers, and based on that and the orientation determine the order of hopper priority
int hopper_ball_count[4] = {1,1,1,1};
int hopper_priority[4] = {0,1,2,3};
int column_priority[7] = {2,4,1,5,3,6,0};
int column_ball_count[7] = {0,0,0,0,0,0,0};

unsigned long turn_time = 1200000;
unsigned long left_turn_time = 1200000;

unsigned long velocity = 1695000;  
unsigned long reverse_velocity = 1580000 + 115000;
unsigned long projected_time;
unsigned long reset_time; 
float length = 28;
float width = 31;
float hopper_radius = 17;
int approach_distance = 25;
int move_count = 1;
int reset_flag = 0;

char player_one_colour = 'a'; 
char player_two_colour = 'b'; 
int turn = 0;     // Player 1's turn - irrelevent in actual competition against robot
int inputcolumn;
int row;
int temp;
int result;
int player_1_score = 0; 
int player_2_score = 0;
int AI_on = 0;
int checkFF = 0;

int row_horizontal_connects[6] = {0,0,0,0,0,0};
int column_vertical_connects[7] = {0,0,0,0,0,0,0};
int diagonal_line_connects[12] = {-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10};

int* full_flag = (int*)calloc(8,sizeof(int));
char** array = (char**)calloc(6, sizeof(char*));

float vertical_distance; 
float horizontal_distance;
float project_dist;

// Mode determines what the robot is doing at the moment
// 0 - Point navigation approaching hopper 0
// 1 - Point navigation approaching hopper 1
// 2 -   "        "          "      hopper 2
// 3 -   "        "          "      hopper 3
// 4 - Hopper 0 approach
// 5 - Hopper 1 approach
// 6 - Hopper 2 approach
// 7 - Hopper 3 approach
// 8 - Game board point navigation
// 9 - Game board approach
int mode;

struct Hopper{
  int input_info_complete;
  float leg1x;
  float leg2x;
  float leg3x;
  float leg1y;
  float leg2y;
  float leg3y;
  float xcenter;
  float ycenter;      // For corner hoppers, set the 3rd leg position at the wall corner to make a barrier that way
};

// 1 - Bottom left corner
// 2 - Bottom right corner
// 3 - Middle left
// 4 - Middle right
struct Hopper hopper[4] = {  
  {1, 0.0, 2.5, 21.25, 0.0, 21.25, 2.5, 6.0, 6.0},          // Hopper 1
  {1, 160.0, 157.5, 138.75, 0.0, 21.25, 2.5, 154.0, 6.0},   // Hopper 2
  {0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},      // Hopper 3
  {0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}       // Hopper 4
};

float x_hopper_holes[7] = {42.8, 55.1, 67.5, 80, 92.4, 104.9, 117.2};
float y_hopper_holes[4] = {57.6, 79.3, 100.8, 122.3};

void forward();
void reverse();
void left();
void right();
void stop_moving();
void reset();
void disposal_servo(int , int* , int* );
int mode_select(int, int*, int*);
void ultrasound(struct Hopper*, float, float, float, unsigned long , float* );

int connect_sum_ai(char** , char , char , int , int , int* , int* , int* , int* );
int displayboard(char** , int* );
int diagonal_connect(char** , int , int , char , int* , int );
int vertical_connect(char** , int , int , char , int* , int );
int horizontal_connect(char** , int , int , char , int* , int );
int connect_4();



void setup(){
  Serial.begin(9600);
  
  pinMode(enablePin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT); 
  digitalWrite(enablePin, LOW);
  
  pinMode(trigPinTop, OUTPUT);
  pinMode(echoPinTop, INPUT); 
  pinMode(trigPinBottom, OUTPUT);
  pinMode(echoPinBottom, INPUT); 
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT); 
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT); 
  
  myDisposalServo.attach(disposalEnablePin);
  myDisposalServo.write(65);                  // Setting disposal servo to centermost column
  delay(1000);
  
  pinMode(fanEnablePin, OUTPUT);
  digitalWrite(fanEnablePin, LOW);
  myServoClaw.attach(clawServoPin);          // Note: Servo motors automatically go to the 0 position at the beginning of the program (claw closed, gate open)
  myServoGate.attach(gateServoPin);
  myServoClaw.write(0);                      // Close claw
  myServoGate.write(82);                     // Close gate         

  digitalWrite(trigPinTop, LOW);
  digitalWrite(trigPinBottom, LOW);
  digitalWrite(trigPinLeft, LOW);
  digitalWrite(trigPinRight, LOW);
  
   pinMode(column1, OUTPUT);
   pinMode(column2, OUTPUT);
   pinMode(column3, OUTPUT);
   pinMode(column4, OUTPUT);
   pinMode(column5, OUTPUT);
   for (int i = 0; i < 5; i++) {  digitalWrite(columns[i],LOW);}
   
   pinMode(row1, INPUT);
   pinMode(row2, INPUT);
   pinMode(row3, INPUT);
   pinMode(row4, INPUT);
   pinMode(row5, INPUT);
   pinMode(row6, INPUT);
   pinMode(row7, INPUT);
   
   for (int i = 0; i < 8; i++)
   {    full_flag[i] = 0;}

   for (int i = 0; i < 6; i++)
   {    array[i] = (char*)calloc(7, sizeof(char));}

   for (int i = 0; i < 6; i++)
   {    for (int j = 0; j < 7; j++)
       {     array[i][j] = ' ';}
   }
     
   displayboard(array, full_flag);
}

void loop(){ 

  if (reset_flag == 1)
  {  
      Serial.println("RESETTING!!!!!");
      while ((millis() - reset_time) < 90000)  {}
  }
  reset_flag = 0;
  
  int current_row = -1; 
  int current_col = -1;
  int hopper_count = 2;
  
  while((hopper[3]).input_info_complete == 0)
  {
    for (int c = 4; c >= 0; c--)
    {
      // Check to see if all the info was successfully inputted
      Serial.println("Checking if info complete ");
      if ((hopper[2]).input_info_complete == 1) 
      {  hopper_count = 3;
         Serial.println("OK SO HOPPER 3 IS DONE NOW ONTO THE FINAL HOPPER");}      
          
      if ((hopper[3]).input_info_complete == 1) 
      {   break;
          Serial.println("YOU DID IT MAN. HURRAY :D");}      
      
       digitalWrite(columns[c], HIGH);
       if (c == 0)                                            // Reset is he top right button. Once pushed, it goes back and restarts the entire input process
       {  if (digitalRead(rows[0]) == HIGH) 
          { digitalWrite(columns[c], LOW);
            Serial.println("Reset button was pushed. Restarting the input process");
            hopper[2].input_info_complete = 0;
            hopper[2].leg1x = -1;
            hopper[2].leg2x = -1;
            hopper[2].leg3x = -1;
            hopper[2].leg1y = -1;
            hopper[2].leg2y = -1;
            hopper[2].leg3y = -1;
            hopper[3].input_info_complete = 0;
            hopper[3].leg1x = -1;
            hopper[3].leg2x = -1;
            hopper[3].leg3x = -1;
            hopper[3].leg1y = -1;
            hopper[3].leg2y = -1;
            hopper[3].leg3y = -1;
            return;}             // Goes back to the beginning to restart everything
          else 
          {   digitalWrite(columns[c], LOW);
              continue;}                                      // Skips the rest of column 5 and moves onto hopper 4, or (if done 4) goes to compute centers and locations
       }
       for (int r = 6; r >= 0; r--)
       { Serial.println("I'm reading rows now "); 
         while (digitalRead(rows[r]) == HIGH)
          {  Serial.print("SOMETHING WAS PUSHED!  row: ");
             Serial.print(4 - c);
             Serial.print("  col: ");
             Serial.println(6 - r);
             current_row = 6 - r;
             current_col = 4 - c;
          }
       } 
    
       digitalWrite(columns[c], LOW);   
       
       if (current_row > -1)
       {
         Serial.println("I'm in here for x");
         if (hopper[hopper_count].leg1x < 0) (hopper[hopper_count]).leg1x = x_hopper_holes[current_row];
         else if ((hopper[hopper_count]).leg2x < 0) (hopper[hopper_count]).leg2x = x_hopper_holes[current_row];
         else if ((hopper[hopper_count]).leg3x < 0) (hopper[hopper_count]).leg3x = x_hopper_holes[current_row];
                 
         Serial.print("leg1x: ");
         Serial.println(hopper[hopper_count].leg1x);
         Serial.print("leg2x: ");
         Serial.println((hopper[hopper_count]).leg2x);
         Serial.print("leg3x: ");
         Serial.println((hopper[hopper_count]).leg3x);
       }
    
       if (current_col > -1)
       {
         Serial.println("I'm in here for y");
         if ((hopper[hopper_count]).leg1y < 0) (hopper[hopper_count]).leg1y = y_hopper_holes[current_col];
         else if ((hopper[hopper_count]).leg2y < 0) (hopper[hopper_count]).leg2y = y_hopper_holes[current_col];
         else if ((hopper[hopper_count]).leg3y < 0) (hopper[hopper_count]).leg3y = y_hopper_holes[current_col];
         
         Serial.print("leg1y: ");
         Serial.println(hopper[hopper_count].leg1y);
         Serial.print("leg2y: ");
         Serial.println((hopper[hopper_count]).leg2y);
         Serial.print("leg3y: ");
         Serial.println((hopper[hopper_count]).leg3y);
       }
                  
       if (((hopper[hopper_count]).leg3x > -1) && ((hopper[hopper_count]).leg3y > -1)) (hopper[hopper_count]).input_info_complete = 1;
      
       current_row = -1; 
       current_col = -1;
    }
  }
  
  digitalWrite(columns[0], HIGH);              // To ensure that if the reset is pressed, it will be noticed
  if (digitalRead(rows[0]) == HIGH) 
  { stop_moving();
    reset_flag = 1; 
    return;
  }
  
  Serial.println();
  Serial.println("K ALL THE INFO IS IN. NOW I AM GOING TO COMPUTE LITERALLY EVERYTHING");
  Serial.println();
  
     Serial.print("hopper3 leg1x: ");
   Serial.println(hopper[2].leg1x);
   Serial.print("hopper3 leg1y: ");
   Serial.println(hopper[2].leg1y);
     Serial.print("hopper3 leg2x: ");
   Serial.println(hopper[2].leg2x);
   Serial.print("hopper3 leg2y: ");
   Serial.println(hopper[2].leg2y);
      Serial.print("hopper3 leg3x: ");
   Serial.println(hopper[2].leg3x);
   Serial.print("hopper3 leg3y: ");
   Serial.println(hopper[2].leg3y);
   
     Serial.print("hopper4 leg1x: ");
   Serial.println(hopper[3].leg1x);
   Serial.print("hopper4 leg1y: ");
   Serial.println(hopper[3].leg1y);
      Serial.print("hopper4 leg2x: ");
   Serial.println(hopper[3].leg2x);
   Serial.print("hopper4 leg2y: ");
   Serial.println(hopper[3].leg2y);
      Serial.print("hopper4 leg3x: ");
   Serial.println(hopper[3].leg3x);
   Serial.print("hopper4 leg3y: ");
   Serial.println(hopper[3].leg3y);
   
   hopper[2].xcenter = (hopper[2].leg1x + hopper[2].leg2x + hopper[2].leg3x)/3;
   hopper[2].ycenter = (hopper[2].leg1y + hopper[2].leg2y + hopper[2].leg3y)/3;
   
   Serial.print("hopper3 xcenter: ");
   Serial.println(hopper[2].xcenter);
   Serial.print("hopper3 ycenter: ");
   Serial.println(hopper[2].ycenter);
      
   hopper[3].xcenter = (hopper[3].leg1x + hopper[3].leg2x + hopper[3].leg3x)/3;
   hopper[3].ycenter = (hopper[3].leg1y + hopper[3].leg2y + hopper[3].leg3y)/3;
   
   Serial.print("hopper4 xcenter: ");
   Serial.println(hopper[3].xcenter);
   Serial.print("hopper4 ycenter: ");
   Serial.println(hopper[3].ycenter);
   
   delay(5000);

  if (digitalRead(rows[0]) == HIGH) 
  { stop_moving();
    reset_flag = 1; 
    return;
  }
  
  float targetDistance;
  float initialTime, finalTime;
  float target_angle;

  float angle = 0;
    
  Serial.println("Just waiting at the moment. Please turn on power safely now...");
  delay(5000);
  
  xposition = 80;
  Serial.print("xposition: ");
  Serial.println(xposition);
  yposition = 30;
  Serial.print("yposition: ");
  Serial.println(yposition);

  Serial.print("Front angle: ");
  Serial.println(angle);

  float box_top = 0;
  float box_bottom = 180; 
  float box_left = 160; 
  float box_right = 0;
  for (int j = 2; j < 4; j++)
  {
    if (hopper[j].leg1x < box_left) box_left = hopper[j].leg1x;
    if (hopper[j].leg2x < box_left) box_left = hopper[j].leg2x;
    if (hopper[j].leg3x < box_left) box_left = hopper[j].leg3x;

    if (hopper[j].leg1x > box_right) box_right = hopper[j].leg1x;
    if (hopper[j].leg2x > box_right) box_right = hopper[j].leg2x;
    if (hopper[j].leg3x > box_right) box_right = hopper[j].leg3x;

    if (hopper[j].leg1y > box_top) box_top = hopper[j].leg1y;
    if (hopper[j].leg2y > box_top) box_top = hopper[j].leg2y;
    if (hopper[j].leg3y > box_top) box_top = hopper[j].leg3y;

    if (hopper[j].leg1y < box_bottom) box_bottom = hopper[j].leg1y;
    if (hopper[j].leg2y < box_bottom) box_bottom = hopper[j].leg2y;
    if (hopper[j].leg3y < box_bottom) box_bottom = hopper[j].leg3y;
  }
  
  Serial.print("box top: ");
  Serial.println(box_top);
  Serial.print("box bottom: ");
  Serial.println(box_bottom);
  Serial.print("box left: ");
  Serial.println(box_left);
  Serial.print("box right: ");
  Serial.println(box_right);
  
  if (((hopper[2].leg1y > hopper[2].ycenter) + (hopper[2].leg2y > hopper[2].ycenter) + (hopper[2].leg3y > hopper[2].ycenter)) == 2)  {  hopper_priority[2] = 2; hopper_priority[3] = 3;}
  else if (((hopper[3].leg1y > hopper[3].ycenter) + (hopper[3].leg2y > hopper[3].ycenter) + (hopper[3].leg3y > hopper[3].ycenter)) == 2)  {  hopper_priority[2] = 3; hopper_priority[3] = 2;}
  
  if (hopper[2].ycenter >= hopper[3].ycenter)  {hopper_priority[0] = 0; hopper_priority[1] = 1;}
  else {hopper_priority[1] = 0; hopper_priority[0] = 1;}
  
  Serial.print("Hopper priority: ");
  for (int i = 0; i < 4; i++)  {  Serial.print(hopper_priority[i]); Serial.print(" ");}
  Serial.println();

  for (int n = 0; n < 4; n++)
  {    if (hopper_ball_count[hopper_priority[n]] > 0) 
       {  mode = hopper_priority[n];
          break;}
  }
  
  Serial.print("mode: ");
  Serial.println(mode);

  MODES:
  if (digitalRead(rows[0]) == HIGH) 
  { stop_moving();
    reset_flag = 1; 
    return;
  }
  angle = 0;  

  if (mode == 8)       // Go to game board approach site
  {  Serial.println("In mode 8 target selection");
     TargetX = 80;
     TargetY = 155;}
     
  else if (mode == 0)  // Go to 1st hopper
  {  Serial.println("In mode 0 target selection");
     TargetX = 6 + approach_distance;
     TargetY = 6 + approach_distance;}
  else if (mode == 1)  // Go to 2nd hopper
  {  Serial.println("In mode 1 target selection");
     TargetX = 154 - approach_distance;
     TargetY = 6 + approach_distance;}
     
  else if (mode == 2)  
  {  Serial.println("In mode 2 target selection");
     // If in config 1: 3 legs form upward facing triangle, only 1 leg above the center point
     if (((hopper[2].leg1y > hopper[2].ycenter) + (hopper[2].leg2y > hopper[2].ycenter) + (hopper[2].leg3y > hopper[2].ycenter)) == 1) 
     {  Serial.println("Upwards facing triangle");
        if (hopper[2].xcenter < hopper[3].xcenter)   TargetX = hopper[2].xcenter - approach_distance*sin(pi/3);
        else TargetX = hopper[2].xcenter + approach_distance*sin(pi/3);
        TargetY = hopper[2].ycenter + approach_distance*cos(pi/3);
     }
     
     else   // Config 2: downward facing triangle with 2 legs above the center point
     {  Serial.println("Downwards facing triangle");
        TargetX = hopper[2].xcenter;
        TargetY = box_top + 18;
        approach_distance = TargetY - hopper[2].ycenter;  // - hopper[2].ycenter + 20;
     }
  }
  
  else if (mode == 3)  
  {  Serial.println("In mode 3 target selection");
     // If in config 1: 3 legs form upward facing triangle, only 1 leg above the center point
     if (((hopper[3].leg1y > hopper[3].ycenter) + (hopper[3].leg2y > hopper[3].ycenter) + (hopper[3].leg3y > hopper[3].ycenter)) == 1)
     {
        if (hopper[2].xcenter < hopper[3].xcenter)   TargetX = hopper[3].xcenter + approach_distance*sin(pi/3);
        else TargetX = hopper[3].xcenter - approach_distance*sin(pi/3);
        TargetY = hopper[3].ycenter + approach_distance*cos(pi/3);
     }
     
     else   // Config 2: downward facing triangle with 2 legs above the center point
     {  TargetX = hopper[3].xcenter;
        approach_distance = box_top - hopper[3].ycenter + 20;
        TargetY = hopper[3].ycenter + approach_distance;
     }
  }
  
  else if ((mode >= 4) && (mode <= 7))
  {   Serial.println("In mode 4-7 target selection");
      
      if (mode == 4)  {target_angle = 45; Serial.println("In mode 4 target selection");}
      else if (mode == 5)  {target_angle = 315; Serial.println("In mode 5 target selection");}
      
      else if (((hopper[mode - 4].leg1y > hopper[mode - 4].ycenter) + (hopper[mode - 4].leg2y > hopper[mode - 4].ycenter) + (hopper[mode - 4].leg3y > hopper[mode - 4].ycenter)) == 1)
      {    if (hopper[mode - 4].ycenter > TargetY)  target_angle = 180;      // Should never run into this
           else if ((hopper[mode - 4].xcenter > TargetX) && (hopper[mode - 4].ycenter < TargetY))  target_angle = 300;
           else if ((hopper[mode - 4].xcenter < TargetX) && (hopper[mode - 4].ycenter < TargetY))  target_angle = 60;}
     
      else if (((hopper[mode - 4].leg1y > hopper[mode - 4].ycenter) + (hopper[mode - 4].leg2y > hopper[mode - 4].ycenter) + (hopper[mode - 4].leg3y > hopper[mode - 4].ycenter)) == 2)
      {    if (hopper[mode - 4].ycenter < TargetY)  target_angle = 0;
           else if ((hopper[mode - 4].xcenter > TargetX) && (hopper[mode - 4].ycenter > TargetY))  target_angle = 240;    // Should never run into this or the next one
           else if ((hopper[mode - 4].xcenter < TargetX) && (hopper[mode - 4].ycenter > TargetY))  target_angle = 120;}  // TargetY should always be above hopper ycenter, this is just to be safe
           
      else {Serial.print("Weird case. Mode is: "); Serial.println(mode);}

      Serial.print("Angle: ");
      Serial.println(angle);
      Serial.print("TargetX: ");
      Serial.println(TargetX);
      Serial.print("TargetY: ");
      Serial.println(TargetY);
      Serial.print("Target angle: ");
      Serial.println(target_angle);

      if (target_angle <= 180)
      {   initialTime = micros();
          right();
          while((micros() - initialTime) < (turn_time*(abs(target_angle)/90))) {}  
          stop_moving();
          delay(300);
      } 
        
      else 
       {  initialTime = micros();
          left();
          while((micros() - initialTime) < (left_turn_time)*(360 - target_angle)/90) {}  
          stop_moving();
          delay(300);
       }
      
       myServoClaw.write(172);                // Open claw
       
       initialTime = micros();
       digitalWrite(leftPin, LOW);        // Using PWM on enable pin to move at half speed (MAY NEED TESTING AROUND) for slower and safer approach
       digitalWrite(rightPin, LOW);       // PWM maps values from 0 - 255. 0 LOW, 255 HIGH. 127 is mid, so if its linear it should move the bot backwards at half speed
       digitalWrite(enablePin, HIGH);       // Reverse to back into the hopper
       Serial.println("Moving Backwards");
       
       while((micros() - initialTime) < velocity*(approach_distance)/50) {}  
       stop_moving();
       delay(300); 
      
       myServoClaw.write(0);          // Close the claw to grab the ball
       delay(750);                    // Short delay to wait for claw to bring the ball in
       Serial.println("I GOT THE BALL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");       
       
       initialTime = micros();
       forward();              // Return back to the initial hopper approach point
       while((micros() - initialTime) < velocity*(approach_distance - 10)/50) {}  
       stop_moving(); 
       delay(300);       
    
      // Now turn to face up again
       if (target_angle <= 180)
       {  initialTime = micros();
          left();
          while((micros() - initialTime) < (left_turn_time)*(target_angle)/90) {}  
          stop_moving();
          delay(300);
       }
       
       else
       {  initialTime = micros();
          right();
          while((micros() - initialTime) < turn_time*abs(360 - target_angle)/90) {}  
          stop_moving();
          delay(300);
       }
       
       
       if (mode == 4)
       {   
          Serial.println("Moving to the side more to avoid the hopper corners (left, forward, right to face 0)"); 
         
          initialTime = micros();
          left();
          while((micros() - initialTime) < left_turn_time) {}  
          stop_moving();
          delay(100);
          
          initialTime = micros();
          forward();              // Return back to the initial hopper approach point
          while((micros() - initialTime) < velocity*(20)/50) {}  
          stop_moving();
          delay(100);
          
          initialTime = micros();
          right();
          while((micros() - initialTime) < turn_time*abs(90)/90) {}  
          stop_moving();
          delay(100);
          
          xposition -= 20;
       }
       
       else if (mode == 5)
       {   
          Serial.println("Moving to the side more to avoid the hopper corners (right, forward, left to face 0)"); 
         
          initialTime = micros();
          right();
          while((micros() - initialTime) < turn_time*(90)/90) {}  
          stop_moving();
          delay(100);
          
          initialTime = micros();
          forward();              // Return back to the initial hopper approach point
          while((micros() - initialTime) < velocity*(20)/50) {}  
          stop_moving();
          delay(100);
          
          initialTime = micros();
          left();
          while((micros() - initialTime) < (left_turn_time)*abs(90)/90) {}  
          stop_moving();
          delay(100);
          
          xposition += 20;
       }  
       
       approach_distance = 37;
       goto NEXT_MODE_SELECT;        // Done the retrieval process, so go back to the mode function to pick the next target point 
  }
    else if (mode == 9)        // Game board approach
    {
       myServoGate.write(82);               // Make sure the gate is closed (if not, close it)
       digitalWrite(fanEnablePin, HIGH);    // Turn on the fan to bring the ball up
       
       initialTime = micros();
       digitalWrite(leftPin, HIGH);        // Using PWM to move at half speed (MAY NEED TESTING AROUND) for slower and safer approach
       digitalWrite(rightPin, HIGH);
       digitalWrite(enablePin, HIGH);      
       Serial.println("Moving forward");
       
       while((micros() - initialTime) < velocity*(10)/50) {}  
       stop_moving();
       delay(100);

       digitalWrite(fanEnablePin, LOW);       
       disposal_servo(move_count, column_priority, column_ball_count);
       move_count++;              
       
       initialTime = micros();
       reverse();              // Reverse back to the initial hopper approach point
       while((micros() - initialTime) < reverse_velocity*(30 - length/2)/50) {}  
       stop_moving();
       delay(100);
       
       goto NEXT_MODE_SELECT;        // Done the retrieval process, so go back to the mode function to pick the next target point
    }


  // If it is doing an approach mode, then angle in the direction of the exact target, slowly go there (PWM), and then call retrieval or disposal functions
  // Once that is done then goto NEXT_MODE_SELECT;
  
  LOOP:
  if (digitalRead(rows[0]) == HIGH) 
  { stop_moving();
    reset_flag = 1; 
    return;
  }
  
  Serial.print("TargetX: ");
  Serial.println(TargetX);
  Serial.print("TargetY: ");
  Serial.println(TargetY);

  // If the robot and the target point are on the same side of the hopper box
  if (((xposition > box_right)&&(TargetX > box_right))||((xposition < box_left)&&(TargetX < box_left))||((yposition > box_top)&&(TargetY > box_top))||((yposition < box_bottom)&&(TargetY < box_bottom))) 
  {
    Serial.println("Same side of the box");
    vertical_distance = (TargetY - yposition);            // Distance from the robot to the top wall
    horizontal_distance = (TargetX - xposition);
    Serial.print("Vertical distance :");
    Serial.println(vertical_distance);
    Serial.print("Horizontal distance :");
    Serial.println(horizontal_distance);
    
    if (TargetY < yposition)
    {  if (angle == 0) 
       {  initialTime = micros();
          reverse();
          while ((micros() - initialTime) < abs(vertical_distance)*reverse_velocity/50) {}
          stop_moving();
          Serial.println("Moved backwards to the right y position");
          angle = 0;
       }
       
       else
       {
         if (angle > 180)
        {  initialTime = micros();
           left();
           while ((micros() - initialTime) < left_turn_time*abs(angle - 180)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned left to go down to target");
        }
        
        else
        {  initialTime = micros();
           right();
           while ((micros() - initialTime) < turn_time*abs(180 - angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned right to go down to target");
        }
        
        angle = 180;
        initialTime = micros();
        forward();
        while ((micros() - initialTime) < abs(vertical_distance)*velocity/50) {}
        stop_moving();
        Serial.println("Moved forwards to the right y position");
       }
    }
    
    else 
    {   
      if (angle == 180)
      {
          initialTime = micros();
          reverse();
          while ((micros() - initialTime) < abs(vertical_distance)*reverse_velocity/50) {}
          stop_moving();
          Serial.println("Moved backwards to the right y position");
          angle = 180;
      }
      
      else
      {  if (angle < 180)
        {  initialTime = micros();
           left();
           while ((micros() - initialTime) < left_turn_time*abs(angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned left to go up to target");
        }
        
        else
        {  initialTime = micros();
           right();
           while ((micros() - initialTime) < turn_time*abs(360 - angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned right to go up to target");
        }
        
        angle = 0;
        
        initialTime = micros();
        forward();  
        while ((micros() - initialTime) < abs(vertical_distance)*velocity/50) {}
        stop_moving();
        Serial.println("Moved forwards to the right y position");
      }
    }
    
    if (TargetX < xposition)
    {  if (angle == 90)
       {
          initialTime = micros();
          reverse();
          while ((micros() - initialTime) < abs(horizontal_distance)*reverse_velocity/50) {}
          stop_moving();
          Serial.println("Moved backwards to the right x position");
          angle = 90;
       }
     
       else
       {
        if (angle > 270)
        {  initialTime = micros();
           left();
           while ((micros() - initialTime) < left_turn_time*abs(270 - angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned left to go left to target");
        }
        
        else if (angle < 90)
        {  initialTime = micros();
           left();
           while ((micros() - initialTime) < left_turn_time*abs(90 + angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned left to go left to target");
        }
        
        else
        {  initialTime = micros();
           right();
           while ((micros() - initialTime) < turn_time*abs(270 - angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned right to go left to target");
        }
        
        angle = 270;
        initialTime = micros();
        forward();
        while ((micros() - initialTime) < abs(horizontal_distance)*velocity/50) {}
        stop_moving();
        Serial.println("Moved forwards to the right x position");
       }
    }
    
    else 
    { if (angle == 270)
      {
          initialTime = micros();
          reverse();
          while ((micros() - initialTime) < abs(horizontal_distance)*reverse_velocity/50) {}
          stop_moving();
          Serial.println("Moved backwards to the right x position");
          angle = 270;
      }
        
      else
      {
        if (angle > 270)
        {  initialTime = micros();
           right();
           while ((micros() - initialTime) < turn_time*abs(360 - angle + 90)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned right to go right to target");
        }
        
        else if (angle < 90)
        {  initialTime = micros();
           right();
           while ((micros() - initialTime) < turn_time*abs(90 - angle)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned right to go right to target");
        }
        
        else
        {  initialTime = micros();
           left();
           while ((micros() - initialTime) < turn_time*abs(angle - 90)/90) {}
           stop_moving();
           delay(300);
           Serial.println("Turned left to go right to target");
        }
        
        angle = 90;
        initialTime = micros();
        forward();
        while ((micros() - initialTime) < abs(horizontal_distance)*velocity/50) {}
        stop_moving();
        Serial.println("Moved forwards to the right x position");
      }
    }
    
    Serial.println("Now facing angle: ");
    Serial.println(angle);

   if (angle <= 180)
    {  initialTime = micros();              
       left();
       while ((micros() - initialTime) < (left_turn_time)*(abs(angle))/90) {}
       stop_moving();
       finalTime = micros();
       delay(300);
       Serial.println("Turn left to 0 to finish point travel");
    }
    
    else
    {  initialTime = micros();              
       right();
       Serial.print("angle: ");
       Serial.println(angle);
       while ((micros() - initialTime) < turn_time*(abs(360 - angle))/90) {}
       stop_moving();
       finalTime = micros();
       delay(300);
       Serial.println("Turn right to 0 to finish point travel");
    }
    
    delay(1500);
    ultrasound(hopper, TargetX, TargetY, 0, projected_time, return_position);
    xposition = return_position[0];
    yposition = return_position[1];
    
    // Account for the deviation from the target point caused by imperfect timed motion 
    if ((yposition - TargetY) > 5)
    {  
        Serial.println("Accounting for deviation from the target point: too high going down");
        initialTime = micros();
        reverse();
        while((micros() - initialTime) < reverse_velocity*abs(yposition - TargetY)/50) {}
        stop_moving();
        delay(300);
    }
    
    else if ((TargetY - yposition) > 5)
    {  
        Serial.println("Accounting for deviation from the target point: too low going up");
        initialTime = micros();
        forward();
        while((micros() - initialTime) < velocity*abs(TargetY - yposition)/50) {}
        stop_moving();
        delay(300);
    }
    
    if ((xposition - TargetX) > 5)
    {
        Serial.println("Accounting for deviation from the target point: too right going left");
        initialTime = micros();
        left();
        while((micros() - initialTime) < left_turn_time) {}
        stop_moving();
        delay(300);
      
        initialTime = micros();
        forward();
        while((micros() - initialTime) < velocity*abs(xposition - TargetX)/50) {}
        stop_moving();
        delay(300); 
   
        initialTime = micros();
        right();
        while((micros() - initialTime) < turn_time) {}
        stop_moving();
        delay(300);     
    }
    
    else if ((TargetX - xposition) > 5)
    {
        Serial.println("Accounting for deviation from the target point: too left going right");
        initialTime = micros();
        right();
        while((micros() - initialTime) < turn_time) {}
        stop_moving();
        delay(300);
      
        initialTime = micros();
        forward();
        while((micros() - initialTime) < velocity*abs(TargetX - xposition)/50) {}
        stop_moving();
        delay(300); 
   
        initialTime = micros();
        left();
        while((micros() - initialTime) < left_turn_time) {}
        stop_moving();
        delay(300);     
    }

    ultrasound(hopper, TargetX, TargetY, 0, projected_time, return_position);
    xposition = return_position[0];
    yposition = return_position[1];

    Serial.print("Got to target position (");
    Serial.print(xposition);
    Serial.print(",");
    Serial.print(yposition);
    Serial.println(")");
    delay(2500);
    
    
    NEXT_MODE_SELECT:
    
    mode = mode_select(mode, hopper_ball_count, hopper_priority);
    Serial.print("Mode: ");
    Serial.print(mode); 
    Serial.print("    Ball count: ");
    for (int m = 0; m < 4; m++)  {  Serial.print(hopper_ball_count[m]); Serial.print(" ");}
    Serial.println();

    goto MODES;               // Done so just restart the program
  }


  else              // Not on the same side of the box as the Target point
  {
    float next_target = 0;

    // For as long as they are on different sides of the box
    while (!(((xposition > box_right)&&(TargetX > box_right))||((xposition < box_left)&&(TargetX < box_left))||((yposition > box_top)&&(TargetY > box_top))||((yposition < box_bottom)&&(TargetY < box_bottom))))
    {
      Serial.println("Different sides of the box");
      // If the robot is located beneath the box
      if (yposition < box_bottom)
      {             
        Serial.println("Robot is below the box");
        if ((xposition > box_left) && (xposition < box_right))       // Stuck under the box so needs to go left or right before going up
        {   
          Serial.println("Stuck below box");
          if (TargetX <= 80)                                       // If the target is in the last half of the board, go left
          {   
            initialTime = micros();
            if ((angle >= 270)||(angle <= 90))         // Facing up, turn left
            {  
              left();
              while ((micros() - initialTime) < (left_turn_time)*((angle + 90)*(angle <= 90) + (angle - 270)*(angle >= 270))/90) {} 
              stop_moving();
              delay(300);
                Serial.println("Turned left ");
            }  

            initialTime = micros();
            if ((angle  > 90) && (angle < 270))        // Facing down, turn right
            {   
              right();
              while((micros() - initialTime) < turn_time*(270 - angle)/90)  {}
              stop_moving();
              delay(300);
                Serial.println("Turned right");
            }

            angle = 270;
            Serial.print("turned to angle: ");
            Serial.println(angle);
            next_target = 0.5*(box_left);                      // Set point at the bottom-left corner, and drive towards it then stop

            targetDistance = abs(next_target - xposition);    // Distance to the point
            projected_time = velocity*targetDistance/50;
            initialTime = micros();
            forward();
            ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
            finalTime = micros();
            stop_moving();
            xposition = return_position[0];
            yposition = return_position[1];  
          
            Serial.print("Moved ");
            Serial.print(targetDistance);
            Serial.print("cm for ");
            Serial.print((finalTime - initialTime)/1000000);
            Serial.println(" seconds");
            Serial.print("x is: ");
            Serial.print(xposition);
            Serial.print(" y is: ");
            Serial.println(yposition);
          }

          else                          // If target is to the right of the center line, go right around the box

          {   
            initialTime = micros();
            if ((angle >= 270)||(angle <= 90))         // Facing up, turn right
            {  
              right();
              Serial.println("Turning right");
              while ((micros() - initialTime) < turn_time*((90 - angle)*(angle <= 90) + (360 - angle + 90)*(angle >= 270))/90)  {}
              stop_moving();
              finalTime = micros();
              delay(300);
            }  

            initialTime = micros();
            if ((angle  > 90) && (angle < 270))        // Facing down, turn left
            {   
              left();
              while((micros() - initialTime) < (left_turn_time)*(angle - 90)/90)  {}
              stop_moving();
              finalTime = micros();
              delay(300);
              Serial.println("Turning left");
            }

            angle = 90;
            next_target = box_right + 0.5*(160 - box_right);                      // Set point at the bottom-right corner, and drive towards it then stop

            targetDistance = abs(next_target - xposition);      // Distance to the point
            projected_time = velocity*targetDistance/50;
            initialTime = micros();
            forward();
            ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
            stop_moving();
            finalTime = micros();
            xposition = return_position[0];
            yposition = return_position[1];
            
            Serial.print("Turned to angle: ");
            Serial.println(angle);
            Serial.print("Moved ");
            Serial.print(targetDistance);
            Serial.print("cm for ");
            Serial.print((finalTime - initialTime)/1000000);
            Serial.println(" seconds");

            Serial.print("x is: ");
            Serial.print(xposition);
            Serial.print(" y is :");
            Serial.println(yposition);
          }
          
          if (((xposition < box_left) && (TargetX < box_left))||((xposition > box_right) && (TargetX > box_right)))  goto LOOP;
        }


        // Beneath box, but in the corner, so can move up
        if ((xposition < box_left) || (xposition > box_right))        
        {  
          Serial.println("Below the box, but in the corner");
          while (angle < 0) angle += 360;
          while (angle >= 360) angle -= 360;
          
          // In a corner, but the target point is at box height, but to the left or right of the box. Go horizontally not vertically in this case
          if (((xposition < box_left)&&((yposition > box_bottom)||(yposition < box_top))) && (TargetX > box_right))
          {   Serial.println("MOVING LATERALLY RIGHT TO THE TARGET POINT ON THE OPPOSITE SIDE (NOT ABOVE OR BELOW)"); 
              if ((angle <= 90) || (angle >= 270))
              {  Serial.println("Turn right to face right");
                 initialTime = micros();  
                 right();
                 while((micros() - initialTime) < turn_time*((angle >= 270)*(360 - angle) + (angle <= 90)*(90 - angle))/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              else 
              {  Serial.println("Turn left to face right");
                 initialTime = micros();  
                 left();
                 while((micros() - initialTime) < (left_turn_time)*(angle - 90)/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              angle = 90;
              next_target = box_right + (160 - box_right)/2;
              targetDistance = abs(next_target - xposition);      // Distance to the point
              projected_time = velocity*targetDistance/50;
              initialTime = micros();
              forward();
              ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
              stop_moving();
              finalTime = micros();
              xposition = return_position[0];
              yposition = return_position[1];
            
              Serial.print("Turned to angle: ");
              Serial.println(angle);
              Serial.print("Moved ");
              Serial.print(targetDistance);
              Serial.print("cm for ");
              Serial.print((finalTime - initialTime)/1000000);
              Serial.println(" seconds");
    
              Serial.print("x is: ");
              Serial.print(xposition);
              Serial.print(" y is: ");
              Serial.println(yposition);
              goto LOOP;
          }
          
          
          else if (((xposition > box_right)&&((yposition > box_bottom)||(yposition < box_top))) && (TargetX < box_left))
          {   Serial.println("MOVING LATERALLY LEFT TO THE TARGET POINT ON THE OPPOSITE SIDE (NOT ABOVE OR BELOW)"); 
              if ((angle <= 90) || (angle >= 270))
              {  Serial.print("Turn left to face left");
                 initialTime = micros();  
                 left();
                 while((micros() - initialTime) < (left_turn_time)*((angle >= 270)*(360 - angle) + (angle <= 90)*(90 - angle))/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              else 
              {  Serial.print("Turn right to face left");
                 initialTime = micros();  
                 right();
                 while((micros() - initialTime) < turn_time*(angle - 90)/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              angle = 270;
              next_target = (box_right)/2;
              targetDistance = abs(next_target - xposition);      // Distance to the point
              projected_time = velocity*targetDistance/50;
              initialTime = micros();
              forward();
              ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
              stop_moving();
              finalTime = micros();
              xposition = return_position[0];
              yposition = return_position[1];
            
              Serial.print("Turned to angle: ");
              Serial.println(angle);
              Serial.print("Moved ");
              Serial.print(targetDistance);
              Serial.print("cm for ");
              Serial.print((finalTime - initialTime)/1000000);
              Serial.println(" seconds");
    
              Serial.print("x is: ");
              Serial.print(xposition);
              Serial.print(" y is: ");
              Serial.println(yposition);
              goto LOOP;
          }
      
          // If it is above or below the box
          if ((angle > 180)&&(angle < 360))               // If facing left, turn right to face up
          {   
            initialTime = micros();  
            right();
            while((micros() - initialTime) < turn_time*(360 - angle)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning right");
          }

          if ((angle <= 180)&&(angle >= 0))               // If facing right, turn left to face up
          {  
            initialTime = micros();  
            left();
            while((micros() - initialTime) < (left_turn_time)*(angle)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning left");
          }

          angle = 0;                

          next_target = box_top + (180 - box_top)/2;

          targetDistance = abs(next_target - yposition);      // Distance to the point
          projected_time = velocity*targetDistance/50;
          initialTime = micros();
          forward();
          ultrasound(hopper, xposition, next_target, angle, projected_time, return_position);
          stop_moving();
          finalTime = micros();
          xposition = return_position[0];
          yposition = return_position[1];
          
          Serial.print("Turned to angle: ");
          Serial.println(angle);
          Serial.print("Moved ");
          Serial.print(targetDistance);
          Serial.print("cm for ");
          Serial.print((finalTime - initialTime)/1000000);
          Serial.println(" seconds");

          Serial.print("x is: ");
          Serial.print(xposition);
          Serial.print(" y is: ");
          Serial.println(yposition);
        }

        goto LOOP;          // Re-iterate through the loop - it should now be ready to directly approach the point
      }



      else if (yposition > box_top)
      {  
        Serial.println("above the box");
        if ((xposition > box_left) && (xposition < box_right))       // Stuck above the box so needs to go left or right before going down
        {
          Serial.println("Stuck above box");
          if (TargetX <= 80)                                       // If the target is in the last half of the board, go left
          {   Serial.print("DEBUG! ANGLE IS: ");
              Serial.println(angle);
            initialTime = micros();
            if ((angle >= 270)||(angle <= 90))         // Facing up, turn left
            {  
              left();
              while ((micros() - initialTime) < (left_turn_time)*((angle + 90)*(angle <= 90) + (angle - 270)*(angle >= 270))/90)  {}
              stop_moving();
              delay(300);
                Serial.println("Turning left");
            }  

            initialTime = micros();
            if ((angle  > 90) && (angle < 270))        // Facing down, turn right
            {   
              right();
              while((micros() - initialTime) < turn_time*(abs(270 - angle))/90)  {}
              stop_moving();
              delay(300);
              Serial.println("Turning right");
            }

            angle = 270;

            next_target = 0.5*(box_left);                      // Set point at the top-left corner, and drive towards it then stop

            targetDistance = abs(next_target - xposition);    // Distance to the point
            projected_time = velocity*targetDistance/50;
            initialTime = micros();
            forward();
            ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
            finalTime = micros();
            stop_moving();
            xposition = return_position[0];
            yposition = return_position[1];
          
            Serial.print("Turned to angle: ");
            Serial.println(angle);            
            Serial.print("Moved ");
            Serial.print(targetDistance);
            Serial.print("cm for ");
            Serial.print((finalTime - initialTime)/1000000);
            Serial.println(" seconds");

            Serial.print("x is: ");
            Serial.print(xposition);
            Serial.print(" y is: ");
            Serial.println(yposition);
          }

          else                          // If target is to the right of the center line, go right around the box

          {   
            initialTime = micros();
            if ((angle >= 270)||(angle <= 90))         // Facing up, turn right
            {  
              right();
              while ((micros() - initialTime) < turn_time*((90 - angle)*(angle <= 90) + (360 - angle + 90)*(angle >= 270))/90) {} 
              stop_moving();
              delay(300);
                Serial.print("Turning right");
            }  

            initialTime = micros();
            if ((angle  > 90) && (angle < 270))        // Facing down, turn left
            {   
              left();
              while((micros() - initialTime) < (left_turn_time)*(angle - 90)/90)  {}
              stop_moving();
              delay(300);
                Serial.println("Turning left");
            }

            angle = 90;

            next_target = box_right + 0.5*(160 - box_right);                      // Set point at the top-right corner, and drive towards it then stop

            targetDistance = abs(next_target - xposition);      // Distance to the point
            projected_time = velocity*targetDistance/50;
            initialTime = micros();
            forward();
            ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
            finalTime = micros();
            stop_moving();
            xposition = return_position[0];
            yposition = return_position[1];
            
            Serial.print("Turned to angle: ");
            Serial.println(angle);
            Serial.print("Moved ");
            Serial.print(targetDistance);
            Serial.print("cm for ");
            Serial.print((finalTime - initialTime)/1000000);
            Serial.println(" seconds");

            Serial.print("x is: ");
            Serial.print(xposition);
            Serial.print(" y is: ");
            Serial.println(yposition);
          }
          
          if (((xposition < box_left) && (TargetX < box_left))||((xposition > box_right) && (TargetX > box_right)))  goto LOOP;
        }


        // Above box, but in the corner, so can move down
        if ((xposition < box_left) || (xposition > box_right))        
        {         
          Serial.println("Above the box, but in the corner");
          while (angle < 0) angle += 360;
          while (angle >= 360) angle -= 360;

          // In a corner, but the target point is at box height, but to the left or right of the box. Go horizontally not vertically in this case
          if (((xposition < box_left)&&((yposition > box_bottom)||(yposition < box_top))) && (TargetX > box_right))
          {   Serial.println("MOVING LATERALLY RIGHT TO THE TARGET POINT ON THE OPPOSITE SIDE (NOT ABOVE OR BELOW)"); 
              if ((angle <= 90) || (angle >= 270))
              {  Serial.println("Turn right to face right");
                 initialTime = micros();  
                 right();
                 while((micros() - initialTime) < turn_time*((angle >= 270)*(360 - angle) + (angle <= 90)*(90 - angle))/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              else 
              {  Serial.println("Turn left to face right");
                 initialTime = micros();  
                 left();
                 while((micros() - initialTime) < (left_turn_time)*(angle - 90)/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              angle = 90;
              next_target = box_right + (160 - box_right)/2;
              targetDistance = abs(next_target - xposition);      // Distance to the point
              projected_time = velocity*targetDistance/50;
              initialTime = micros();
              forward();
              ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
              finalTime = micros();
              stop_moving();
              xposition = return_position[0];
              yposition = return_position[1];
              
              Serial.print("Turned to angle: ");
              Serial.println(angle);
              Serial.print("Moved ");
              Serial.print(targetDistance);
              Serial.print("cm for ");
              Serial.print((finalTime - initialTime)/1000000);
              Serial.println(" seconds");
    
              Serial.print("x is: ");
              Serial.print(xposition);
              Serial.print(" y is: ");
              Serial.println(yposition);
              goto LOOP;
          }
          
          
          else if (((xposition > box_right)&&((yposition > box_bottom)||(yposition < box_top))) && (TargetX < box_left))
          {   Serial.println("MOVING LATERALLY LEFT TO THE TARGET POINT ON THE OPPOSITE SIDE (NOT ABOVE OR BELOW)"); 
              if ((angle <= 90) || (angle >= 270))
              {  Serial.print("Turn left to face left");
                 initialTime = micros(); 
                left(); 
                 while((micros() - initialTime) < (left_turn_time)*((angle >= 270)*(360 - angle) + (angle <= 90)*(90 - angle))/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              else 
              {  Serial.print("Turn right to face left");
                 initialTime = micros();  
                 right();
                 while((micros() - initialTime) < turn_time*(angle - 90)/90)  {}
                 stop_moving();
                 delay(300);
              }
              
              angle = 270;
              next_target = (box_right)/2;
              targetDistance = abs(next_target - xposition);      // Distance to the point
              projected_time = velocity*targetDistance/50;
              initialTime = micros();
              forward();
              ultrasound(hopper, next_target, yposition, angle, projected_time, return_position);
              finalTime = micros();
              stop_moving();
              xposition = return_position[0];
              yposition = return_position[1];
            
              Serial.print("Turned to angle: ");
              Serial.println(angle);
              Serial.print("Moved ");
              Serial.print(targetDistance);
              Serial.print("cm for ");
              Serial.print((finalTime - initialTime)/1000000);
              Serial.println(" seconds");
    
              Serial.print("x is: ");
              Serial.print(xposition);
              Serial.print(" y is: ");
              Serial.println(yposition);
              goto LOOP;
          }
          
          // Target is above or below the box, not on the sides 
          if ((angle > 180)&&(angle < 360))               // If facing left, turn left to face down
          {   
            initialTime = micros();  
            left();
            while((micros() - initialTime) < (left_turn_time)*(angle - 180)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning left");
          }

          if ((angle <= 180)&&(angle >= 0))               // If facing right, turn right to face down
          {  
            initialTime = micros();  
            right();
            while((micros() - initialTime) < turn_time*(180 - angle)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning right");
          }

          angle = 180;                

          next_target = (box_bottom + 21.25)/2;

          targetDistance = abs(yposition - next_target);      // Distance to the point
          projected_time = velocity*targetDistance/50;
          initialTime = micros();
          forward();
          ultrasound(hopper, xposition, next_target, angle, projected_time, return_position);
          finalTime = micros();
          stop_moving();
          xposition = return_position[0];
          yposition = return_position[1];
            
          Serial.print("Turned to angle: ");
          Serial.println(angle);
          Serial.print("Moved ");
          Serial.print(targetDistance);
          Serial.print("cm for ");
          Serial.print((finalTime - initialTime)/1000000);
          Serial.println(" seconds");

          Serial.print("x is: ");
          Serial.print(xposition);
          Serial.print(" y is: ");
          Serial.println(yposition);
        }

        goto LOOP;          // Re-iterate through the loop - it should now be ready to directly approach the point
      }





      else if ((xposition < box_left) || (xposition > box_right))       // On either side of the box, but not above or below it
      {
        Serial.println("Beside the box, but not above or below it");
        if (TargetY < yposition)                            // Target point is below the current position
        {
          Serial.println("Above the target position");
          if ((angle > 180)&&(angle < 360))                // Facing left, turn left to go down
          {   
            initialTime = micros(); 
            left(); 
            while((initialTime) < (left_turn_time)*(angle - 180)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning left");
          }

          if ((angle <= 180)&&(angle >= 0))                // Facing right, turn right to go down
          {   
            initialTime = micros(); 
            right(); 
            while((micros() - initialTime) < turn_time*(180 - angle)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning right");
          }

          angle = 180;
          next_target = (box_bottom + 21.25)/2;

          targetDistance = abs(yposition - next_target);      // Distance to the point
          projected_time = velocity*targetDistance/50;
          initialTime = micros();
          forward();
          ultrasound(hopper, xposition, next_target, angle, projected_time, return_position);
          finalTime = micros();
          stop_moving();
          xposition = return_position[0];
          yposition = return_position[1];
            
          Serial.print("Turned to angle: ");
          Serial.println(angle);
          Serial.print("Moved ");
          Serial.print(targetDistance);
          Serial.print("cm for ");
          Serial.print((finalTime - initialTime)/1000000);
          Serial.println(" seconds");

          Serial.print("x is: ");
          Serial.print(xposition);
          Serial.print(" y is: ");
          Serial.println(yposition);
        }  

        else if (TargetY > yposition)                            // Target point is above the current position
        {  
          Serial.println("Bot is below target position");
          if ((angle > 180)&&(angle < 360))                // Facing left, turn right to go up
          {   
            initialTime = micros();  
            right();
            while((micros() - initialTime) < turn_time*(360 - angle)/90) {}
            stop_moving();
            delay(300);
            Serial.println("Turning right");
          }

          if ((angle <= 180)&&(angle >= 0))                // Facing right, turn left to go up
          {   
            initialTime = micros();  
            left();
            while((micros() - initialTime) < (left_turn_time)*(angle)/90)  {}
            stop_moving();
            delay(300);
            Serial.println("Turning left");
          }

          angle = 0;
          next_target = box_top + (180 - box_top)/2;

          targetDistance = abs(yposition - next_target);      // Distance to the point
          projected_time = velocity*targetDistance/50;
          initialTime = micros();
          forward();
          ultrasound(hopper, xposition, next_target, angle, projected_time, return_position);
          finalTime = micros();
          stop_moving();
          xposition = return_position[0];
          yposition = return_position[1];
          
          Serial.print("Turned to angle: ");
          Serial.println(angle);
          Serial.print("Moved ");
          Serial.print(targetDistance);
          Serial.print("cm for ");
          Serial.print((micros() - initialTime)/1000000);
          Serial.println(" seconds");

          Serial.print("x is: ");
          Serial.print(xposition);
          Serial.print(" y is: ");
          Serial.println(yposition);
        }   
        goto LOOP;        
      }

      else
      {  
        Serial.println("Robot thinks it's in the hopper region. If this problem continues, please place the robot elsewhere and restart");
        goto LOOP;
      }
    }

  }
}




// Coordinate and Position system

// Keep track of x and y coordinates
// Origin of board is bottom left corner (with respect to the game board being at the top of the coordinate plane)
// Get hopper leg locations inputed and store them as coordinates
// Compute radius/box, find center and input it as coordinate
// Determine which ultrasound sensors to use
// Call ultrasound code to determine distance to the two walls
// Use distances to determine position coordinates
// Angle 0 degrees is pointing directly at the game board


void ultrasound(struct Hopper* hopper, float input_xposition, float input_yposition, float front_angle, unsigned long travel_time, float* coordinates)
{
  // Need to keep track of time to see when it believes it is in position and should shoot off the ultrasound sensors to find actal position
  unsigned long time_count = micros();
  Serial.println();
  Serial.println("Ultrasound portion: ");
  Serial.println();
  
  Serial.print("inputted x: ");
  Serial.println(input_xposition);
  Serial.print("inputted y: ");
  Serial.println(input_yposition);
  
  if ((micros() - time_count) >= travel_time) stop_moving();
  
  float time1, time2, xdistance, ydistance, LEC;
  int inputEchoPin[2] = {-1, -1};
  int inputTrigPin[2] = {-1, -1};
  char sensor[2] = {'x', 'x'};
  float angle[4] = {front_angle, front_angle + 180, front_angle + 270, front_angle + 90};
  for (int i = 0; i < 4; i++)
  {    
    while (angle[i] < 0)  angle[i] += 360;
    while (angle[i] >= 360)  angle[i] -= 360;
  }      // Keep angles within the range of 0 to 360

  if ((micros() - time_count) >= travel_time) stop_moving();

    // For using ultrasound to find position. If -1, then not hitting a hopper. Otherwise it tells it which hopper its hitting
  int hopper_detect[2] = {-1,-1};
  int sensors_selected[2] = {-1, -1};

  // 1st item is top, 2nd is bottom, 3rd is left, 4th is right
  // This keeps track of the estimated state of the sensor: 0 (hits wall far away), 1 (hits hopper far away), 2 (hits wall close beside it), 3 (hits hopper right beside it), 4 (not sure)
  // Higher priority is closer to 0, lower is closer to 3. Helps to determine which sensors have the best current vision and this should be used
  float sensorstate[4] = {0,0,0,0};
  float approx_sensor_distances[4] = {200,200,200,200};  // Keep track of how far away the sensor thinks it is based on preliminary evaluation. Helps comparison to determine sensorstate

  Serial.print("Robot center x position: ");
  Serial.println(xposition);
  Serial.print("Robot center y position: ");
  Serial.println(yposition);

  LOOP:

  // A flag will detect if it is the intial reading
  // Then, to determine the initial coordinates, pick 2 arbitrary sensors (top and left), turn the flag on (for the remainder of the time) and jump to ultrasound measurements
  // The flag will stay on for the rest of this game after that. It is ONLY to determine the initial coordinates

  // For each sensor, see if it is at risk of hitting a hopper using gyro for orientation (vector.h might help)
  // Find the two that are least likely to hit a hopper
  // If there is a tie amongst a few, use the ones that are best in the range (NOT within 10.5cm, and after that i guess the closest as there is less risk of interference)
  // For the two chosen sensors, shoot out a signal and compute location below

  // Read the angle the disposal face is facing from the gyro


  // Once this is done for all 4 sensors, find the best two to use
  //       0 (hits wall far away)                            Higher priority (best option)
  //       1 (hits hopper far away)
  //       2 (hits wall close beside it)
  //       3 (hits hopper right beside it)                   Lower priority (use if the other sensor won't yield better results)
  //       4 (no idea - default)                             Don't use this unless there is nothing else
  // Pick which hopper for x and which hopper for y by comparing the two sensors return vals (above - priority given to the lower return val)


  // Can also say that if distance is <= 10.5cm (minimum threshold), disregard it from that sensor and try to deduce position based on last position component and heading (gyro)

  if ((micros() - time_count) >= travel_time) stop_moving();
  for (int i = 0; i < 4; i++) { 
    sensorstate[i] = 0;
  }    // Reset the sensor states before beginning preliminary sensor priority evaluation

  // Go through the sensors and determine the 2 best ones to use for accurate position measurements
  for (int i = 0; i < 4; i++)          // 0 top, 1 bottom, 2 left, 3 right
  {   
    if ((micros() - time_count) >= travel_time) stop_moving();
    
    Serial.println();
    Serial.print("Sensor: ");
    Serial.println(i);

    float current_angle = angle[i];                         // Find the angle each sensor is facing
    while (current_angle < 0)  current_angle += 360;
    while (current_angle >= 360) {  
      current_angle -= 360;
    }      // Keep angles within the range of 0 to 360

    Serial.print("angle: ");
    Serial.println(angle[i]);
    Serial.println(current_angle);

    float Ax = input_xposition + sin(angle[i]*pi/180)*(i<2)*(length/2) + sin(angle[i]*pi/180)*(i >=2)*width/2;
    float Ay = input_yposition + cos(angle[i]*pi/180)*((i<2)*length/2) + cos(angle[i]*pi/180)*(i >=2)*width/2;
    Serial.print("Sensor position: ");
    Serial.print(Ax);
    Serial.print(" ");
    Serial.println(Ay);
    
    if ((micros() - time_count) >= travel_time) stop_moving();

    float Bx = 0;
    float By = 0;
    // Points on the walls in the direction of the angle (pretty arbitrary - just to avoid undershooting)
    if (current_angle == 0) {Bx = input_xposition; By = 180;}
    else if (current_angle == 90) {Bx = 160; By = input_yposition;}
    else if (current_angle == 180) {Bx = input_xposition; By = 0;}
    else if (current_angle == 270) {Bx = 0; By = input_yposition;}
    else {  Serial.print("At a weird angle...."); Serial.println(current_angle);}


    Serial.print("B point: ");
    Serial.print(Bx);
    Serial.print(" ");
    Serial.println(By);

    if ((micros() - time_count) >= travel_time) stop_moving();

    // compute the euclidean distance between A and B        http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    float LAB = sqrt((Bx-Ax)*(Bx-Ax) + (By-Ay)*(By-Ay));
    Serial.print("LAB: ");
    Serial.println(LAB);
    Serial.print("DIST EST: ");
    Serial.println(LAB);
    // compute the direction vector D from A to B
    float Dx = (Bx-Ax)/LAB;
    float Dy = (By-Ay)/LAB;
    Serial.print("D: ");
    Serial.print(Dx);
    Serial.print(" ");
    Serial.println(Dy);

    for (int j = 0; j < 4; j++)      
    {   
      Serial.print("hopper being evaluated: ");
      Serial.println(j);
      if ((micros() - time_count) >= travel_time) stop_moving();
      
      float Cx = (hopper[j]).xcenter;
      float Cy = (hopper[j]).ycenter;

      Serial.print("Hopper center: ");
      Serial.print(Cx);
      Serial.print(" ");
      Serial.println(Cy);

      // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
      // Compute the value t of the closest point to the circle center (Cx, Cy)
      float t = Dx*(Cx-Ax) + Dy*(Cy-Ay); 
      Serial.print("t: ");
      Serial.println(t);

      // This is the projection of C on the line from A to B.

      // compute the coordinates of the point E on line and closest to C
      float Ex = t*Dx+Ax;
      float Ey = t*Dy+Ay;
           
      Serial.print("Nearest point to C: ");
      Serial.print(Ex);
      Serial.print(" ");
      Serial.println(Ey);

      // If E is not in between A and B, it's facing in the opposite direction and so it is irrelevant
      // LEC is not going to intersect a hopper so just assign it an arbitrarily high value and skip to checking for hopper intersection
      if (!((((Ax <= Bx) && (Ex >= Ax) && (Ex <= Bx)) || ((Ax >= Bx) && (Ex <= Ax) && (Ex >= Bx))) && (((Ay <= By) && (Ey >= Ay) && (Ey <= By)) || ((Ay >= By) && (Ey <= Ay) && (Ey >= By)))))
      {    
        Serial.println("Facing the wrong way, jut don't worry about it it's definitely not hitting a hopper");
        LEC = 201;
        goto HOPPER_RADIUS_CHECK;
      }    

      if ((micros() - time_count) >= travel_time) stop_moving();
    
      // compute the euclidean distance from E to C
      // Large second term is to account for the spread of the ultrasound waves
      
      if ((abs(Cx - Ax) <= (abs(tan(15*pi/180)*abs(Cy - Ay)) + hopper_radius)) && ((angle[i] < 45) || (angle[i] > 315) || (((angle[i] < 225))&&(angle[i] > 135)))) {Serial.println("in hopper hit x range"); LEC = sqrt((Ex-Cx)*(Ex-Cx)+(Ey-Cy)*(Ey-Cy)) - sin(15*pi/180)*sqrt((input_xposition - Ex)*(input_xposition - Ex) + (input_yposition - Ey)*(input_yposition - Ey)); project_dist = abs(Ey - Ay);}
      else if ((abs(Cy - Ay) <= (abs(tan(15*pi/180)*abs(Cx - Ax)) + hopper_radius)) && (((angle[i] > 45) && (angle[i] < 135)) || (((angle[i] > 225))&&(angle[i] < 315)))) {Serial.println("in hopper hit y range");LEC = sqrt((Ex-Cx)*(Ex-Cx)+(Ey-Cy)*(Ey-Cy)) - sin(15*pi/180)*sqrt((input_xposition - Ex)*(input_xposition - Ex) + (input_yposition - Ey)*(input_yposition - Ey)); project_dist = abs(Ex - Ax);}
      else LEC = 200;

      Serial.print("Distance from E to C: ");
      Serial.println(LEC);
      Serial.println(angle[i]);
      Serial.println(abs(Cx - Ax));
      Serial.println(abs(tan(15*pi/180)));
      Serial.println(abs(Cy - Ay));
      Serial.println((abs(Cx - Ax) <= abs(tan(15*pi/180)*abs(Cy - Ay))));
      Serial.println();
      Serial.println(abs(Cy - Ay));
     Serial.println(abs(tan(15*pi/180)));
    Serial.println(abs(Cx - Ax));
    Serial.println((abs(Cy - Ay) <= abs(tan(15*pi/180)*abs(Cx - Ax))) && (((angle[i] > 45) && (angle[i] > 135)) || (((angle[i] > 225))&&(angle[i] > 315))));
    
  HOPPER_RADIUS_CHECK:
      float distance = 0;
      // If there is an intersection - detects the hopper
      if (LEC <= hopper_radius)
      {   
        Serial.print(i);
        Serial.println(" sensor hit a hopper");
        // compute distance from the sensor to the center of the hopper
        distance = sqrt((Ax - Cx)*(Ax - Cx) + (Ay - Cy)*(Ay - Cy)) - hopper_radius;
        Serial.println(Ax);
        Serial.println(Ay);
        Serial.println(Cx);
        Serial.println(Cy);
        Serial.print("distance: ");
        Serial.println(distance);
        Serial.print("x component to hopper: ");
        Serial.println((1 - 2*(Cx < Ax))*(abs(Cx - Ax) - hopper_radius));
        Serial.print("y component to hopper: ");
        Serial.println((1 - 2*(Cy < Ay))*(abs(Cy - Ay) - hopper_radius));

        if (distance <= 15) 
          // 3 overrides 1 since it means one of the hoppers is directly beside the sensor, and will interfere with the data
          // Decimal such that 3 indicates a hopper was hit and the decimal point indicates the hopper that was hit
        {   
          if ((approx_sensor_distances[i] > project_dist) && (sensorstate[i] < 4))
          {  approx_sensor_distances[i] = project_dist;
             sensorstate[i] = 3 + ((float)j)/10;}
        }          
        else 
        { 
          if ((sensorstate[i] < 2)&&(approx_sensor_distances[i] > project_dist)) 
          {   approx_sensor_distances[i] = project_dist;
              sensorstate[i] = 1 + ((float) j)/10;}
        }
      }
      
      else     // Hits a wall
      {  
        Serial.print(i);
        Serial.println(" sensor hit a wall");
        Serial.print("Ay ");
        Serial.println(Ay);
        Serial.println(current_angle);
        // Check to see if the sensors facing x or y directions are within 10.5cm of the wall
        // If so, set the sensor state to 2 (detects wall, but too close for accurate ultrasound reading)
        if (((current_angle >= 45)&&(current_angle < 135)&&(Ax >= 160 - 10.5)) || ((current_angle >= 225)&&(current_angle < 315)&&(Ax <= 10.5)))
        {    
          if (sensorstate[i] < 2) sensorstate[i] = 2;
          Serial.println("Too close to the wall in X");
        }
        else if (((current_angle >= 135)&&(current_angle < 225)&&(Ay <= 10.5)) || (((current_angle < 45)||(current_angle >= 315))&&(Ay >= 180 - 10.5)))
        {    
          if (sensorstate[i] < 2) sensorstate[i] = 2;
          Serial.println("Too close to the wall in Y");
        }    
        else Serial.println("Hit wall far away. Nice");
        //         if ((approx_sensor_distances[i] < 0)&&((sensorstate[i] == 0) || ((int)sensorstate[i] == 2))) approx_sensor_distances[i] += 200;
        // If that sensor has only hit walls so far, keep it at (highest priority). This means that the sensor so far has hit nothing but walls.
      }
    }
    if (sensorstate[i] == 0) approx_sensor_distances[i] = LAB;
    else if ((int)sensorstate[i] == 2) approx_sensor_distances[i] = LAB;
  }

  // Info has been gathered, now to determine which sensor is most likely to get a nice accurate reading
  for (int i = 0; i < 4; i++)
  {  
    if ((micros() - time_count) >= travel_time) stop_moving();
    
    // i is for the iteration of the current sensor
    // If one of the sensors got a better priority rating
    if ((sensorstate[i] - sensorstate[(i+1)%2 + 2*(i > 1)]) < -0.5)
    {  
      goto SENSOR_SELECT;
    }

    // If two have an equal rating, go with the closer one (as long as the distance is greater than the 10.5cm cutoff)
    else if (((sensorstate[i] - sensorstate[(i+1)%2 + 2*(i > 1)]) < 0.5) && ((sensorstate[i] - sensorstate[(i+1)%2 + 2*(i > 1)]) > -0.5))
    {  
      if ((approx_sensor_distances[i] < approx_sensor_distances[(i+1)%2 + 2*(i > 1)]) && (approx_sensor_distances[i] > 10.5))
      {  
        goto SENSOR_SELECT;
      }
      
      else if (approx_sensor_distances[i] == approx_sensor_distances[(i+1)%2 + 2*(i > 1)])
      {
         goto SENSOR_SELECT;
      }

      else continue;
    }

    // If it has a lower priority rating, leave it and let the other sensor pick it up
    else continue;

  SENSOR_SELECT: 

    if ((micros() - time_count) >= travel_time) stop_moving();
    int temp = 0;
    int temp_hopper = -2;
    // Just to facilitate sensor selection by temporarily ignoring the hopper that was hit (eliminate the decimal point)
    // This series of if statements is to see if one of the final sensors chosen was in fact hitting a hopper, and if so, which
    if ((sensorstate[i] >= 1) && (sensorstate[i] < 2))  
    {   
      temp = 1;
      temp_hopper = ((int)(10*(sensorstate[i] - temp)));
      if (sensorstate[i] == 1.3) temp_hopper = 3;
    }

    else if ((sensorstate[i] >= 3) && (sensorstate[i] < 4))  
    {   
      temp = 3;
      temp_hopper = ((int)(10*(sensorstate[i] - temp)));
      if (sensorstate[i] == 3.3) temp_hopper = 3;
    }

    //    else temp = i;
    Serial.println("i, temp, temp hopper, sensor state: ");
    Serial.println(i);
    Serial.println(temp);
    Serial.println(temp_hopper);
    Serial.println(sensorstate[i]);

    if ((micros() - time_count) >= travel_time) stop_moving();

    switch(i)
    {
    case 0: 
      {  
        sensor[0] = 't';
        sensors_selected[0] = i;
        hopper_detect[0] = temp_hopper;
        break;
      }
    case 1:
      {  
        sensor[0] = 'b';
        sensors_selected[0] = i;
        hopper_detect[0] = temp_hopper;
        break;
      }
    case 2: 
      {  
        sensor[1] = 'l';
        sensors_selected[1] = i;
        hopper_detect[1] = temp_hopper;
        break;
      }
    case 3:
      {  
        sensor[1] = 'r';
        sensors_selected[1] = i;
        hopper_detect[1] = temp_hopper;
        break;
      }
    }
  }

  Serial.print("Hoppers detected: ");
  Serial.print(hopper_detect[0]);
  Serial.print(" ");
  Serial.println(hopper_detect[1]);

  Serial.print("Sensor 1 selected: ");
  Serial.print(sensor[0]);
  Serial.print(" - ");
  Serial.println(sensors_selected[0]);
  Serial.print("Sensor 2 selected: ");
  Serial.print(sensor[1]);
  Serial.print(" - ");
  Serial.println(sensors_selected[1]);
  Serial.print("Sensor state:");
  for (int j = 0; j < 4; j++)
  {   
    Serial.print(" ");
    Serial.print(sensorstate[j]);
  }
  Serial.println();
  Serial.print("Sensor distance:");
  for (int j = 0; j < 4; j++)
  {   
    Serial.print(" ");
    Serial.print(approx_sensor_distances[j]);
  }
  Serial.println();

  ULTRASOUND: 

  // call ultrasound sensors
  switch(sensor[0])
  {
  case 't': 
    {  
      inputEchoPin[0] = echoPinTop;
      inputTrigPin[0] = trigPinTop; 
      break;
    }
  case 'b': 
    {  
      inputEchoPin[0] = echoPinBottom; 
      inputTrigPin[0] = trigPinBottom; 
      break;
    }
  case 'l': 
    {  
      inputEchoPin[0] = echoPinLeft;
      inputTrigPin[0] = trigPinLeft; 
      break;
    }
  case 'r': 
    {  
      inputEchoPin[0] = echoPinRight; 
      inputTrigPin[0] = trigPinRight;
      break;
    }
  }

  switch(sensor[1])
  {
  case 't': 
    {  
      inputEchoPin[1] = echoPinTop;
      inputTrigPin[1] = trigPinTop; 
      break;
    }
  case 'b': 
    {  
      inputEchoPin[1] = echoPinBottom; 
      inputTrigPin[1] = trigPinBottom; 
      break;
    }
  case 'l': 
    {  
      inputEchoPin[1] = echoPinLeft;
      inputTrigPin[1] = trigPinLeft; 
      break;
    }
  case 'r': 
    {  
      inputEchoPin[1] = echoPinRight; 
      inputTrigPin[1] = trigPinRight;
      break;
    }
  default: 
    Serial.println("nope");
  }


  // Find position based on ultrasound readings
 
  // have a way to check if the bot is facing vertical or horizontal
  // based on that, choose [i]th sensor (0 top/bottom, 1 left/right)
  // go through with all that shtuff as usual

  // i is for the direction of the sensors (marks vertical or horizontal)
  // If the robot is facing vertically, i = 0. Else i = 1 to indicate which set of sensors is facing horizontally or vertically 
  // i indicates top/bottom set facing vertically, !i indicates top/bottom set facing horizontally
  // "Set" means the index of sensors_selected, hopper_detected, sensor, echoPin, trigPin etc.....    0 - top/bottom, 1 - left/right
  
  
  // While it is waiting for the robot to move for the expected amount of time until it is approx in the target position. Then stop moving and get ultrasound position readings
  while ((micros() - time_count) < travel_time) {}
  stop_moving();
  
  int i;
  if (((angle[0] >= 45) && (angle[0] < 135)) || ((angle[0] >= 225) && (angle[0] < 315)))  i = 1;
  else i = 0;
  Serial.print("i: ");
  Serial.println(i);

  Serial.println();
  Serial.println("BEGINNING ULTRASOUND SECTION:");
  Serial.print("X sensor selected: ");
  Serial.println(sensor[!i]);


  // Computing the x position
  unsigned long delaytime = micros();
  digitalWrite(inputTrigPin[!i], HIGH);
  while(micros()-delaytime < 10){}
  digitalWrite(inputTrigPin[!i],LOW);


  ECHO1:
  if (digitalRead(inputEchoPin[!i]) == LOW) goto ECHO1;
  time1 = micros();                            // Once echo goes HIGH
  while(digitalRead(inputEchoPin[!i]) == HIGH){}
  time2 = micros();                            // Once echo goes LOW


  float Ax = (!i)*(0.5*width*sin(angle[sensors_selected[!i]]*pi/180)) + i*(0.5*length*sin(angle[sensors_selected[!i]]*pi/180));
  
  Serial.print("Ax: ");
  Serial.println(Ax);
  xdistance = (time2 - time1)/58;              // As per User Manual   https://docs.google.com/document/d/1Y-yZnNhMYy7rwhAgyL_pfa39RsB-x2qR4vP8saG73rE/edit 
  Serial.print("xdistance: ");
  Serial.println(xdistance);

  Serial.print("Distance of ");
  Serial.print(sensor[!i]);
  Serial.print(" is: ");
  Serial.println(xdistance);
  float xposition = 0;
  float yposition = 0;
  
  if ((micros() - time_count) >= travel_time) stop_moving();
  
  // Find xposition from xdistance and Ax
  // NOTE: Ax is negative when the sensor selected is facing left, so subtract it in both cases as its own sign handles itself
  if (hopper_detect[!i] > -1)
  {  if (angle[sensors_selected[!i]] < 180) xposition = abs((hopper[hopper_detect[!i]]).xcenter - xdistance - Ax);
     else xposition = (hopper[hopper_detect[!i]]).xcenter + xdistance - Ax;}
  else 
  {  if (angle[sensors_selected[!i]] < 180) xposition = 160 - xdistance - Ax;
     else xposition = xdistance - Ax;}


  delaytime = micros();
  digitalWrite(inputTrigPin[i], HIGH);
  while(micros()-delaytime < 10)  continue;
  digitalWrite(inputTrigPin[i],LOW);


  ECHO2:
  if (digitalRead(inputEchoPin[i]) == LOW) goto ECHO2;
  time1 = micros();                            // Once echo goes HIGH
  while(digitalRead(inputEchoPin[i]) == HIGH){}
  time2 = micros();                            // Once echo goes LOW


  // Find Ay
  float Ay = (!i)*(0.5*length*cos(angle[sensors_selected[i]]*pi/180)) + i*(0.5*width*cos(angle[sensors_selected[i]]*pi/180));

  Serial.print("Ay: ");
  Serial.println(Ay);
  ydistance = (time2 - time1)/58;              // As per User Manual

  if ((micros() - time_count) >= travel_time) stop_moving();

  // Find yposition from ydistance and Ay
  // NOTE: Ay is negative when the sensor selected is facing down, so subtract it in both cases as its own sign handles itself
  if (hopper_detect[i] > -1)
  {  if ((angle[sensors_selected[i]] < 270) && (angle[sensors_selected[i]] > 90)) yposition = (hopper[hopper_detect[i]]).ycenter + ydistance - Ay;
     else yposition = abs((hopper[hopper_detect[i]]).ycenter - ydistance - Ay);}
  else
  {  if ((angle[sensors_selected[i]] < 270) && (angle[sensors_selected[i]] > 90)) yposition = ydistance - Ay;
     else yposition = 180 - ydistance - Ay;}
  
  Serial.print("Distance of ");
  Serial.print(sensor[i]);
  Serial.print(" is: ");
  Serial.println(ydistance);

  digitalWrite(inputTrigPin[0], LOW);
  digitalWrite(inputTrigPin[1], LOW);  

  Serial.print("X coordinate is: ");
  Serial.println(xposition);
  Serial.print("Y coordinate is: ");
  Serial.println(yposition);

  coordinates[0] = xposition;
  coordinates[1] = yposition; 

  Serial.print("X coordinate: ");
  Serial.println(xposition);
  Serial.print("Y coordinate: ");
  Serial.println(yposition);
  
  if ((micros() - time_count) >= travel_time) stop_moving();
  return;
}








      

void forward(){                                
    digitalWrite(leftPin, HIGH);
    digitalWrite(rightPin, HIGH);
    digitalWrite(enablePin, HIGH); 
Serial.println("Moving forward");
    return;
}

void reverse(){
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, LOW);
    digitalWrite(enablePin, HIGH);
 Serial.println("Moving backwards");
    return;
}

void left(){
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, HIGH);
    digitalWrite(enablePin, HIGH);
Serial.println("Turning left");
    return;
}

void right(){
    digitalWrite(leftPin, HIGH);
    digitalWrite(rightPin, LOW);
    digitalWrite(enablePin, HIGH);
Serial.println("Turning right");
    return;
}

void stop_moving(){
    digitalWrite(enablePin, LOW);
Serial.println("Stopping!");
//delay(300);
    return;
}

int mode_select(int mode, int* hopper_ball_count, int* hopper_priority) 
{
    if (mode == -1)  return hopper_priority[0];         // For the first move of the game, go to the 1st priority hopper
    
    if ((mode <= 7) && (mode >= 4))    // Done retrieving the ball. Now going to go to the game board (setting target point to be the board approach point)
    {   hopper_ball_count[mode - 4] -= 1;  
        mode = 8;  
        return mode;}         
    else if (mode == 9)            // Finished board approach, now selecting which hopper to go to next
    {  for (int k = 0; k < 4; k++)
       { if (hopper_ball_count[hopper_priority[k]] > 0)
         { mode = hopper_priority[k];
          return mode;}
       }
       Serial.println("Done the game now - out of balls");
       while (1) {}
    }
    else if (mode == 8)  mode++;        // Reached the approach point for the game board, now begin game board approach
    else if ((mode >= 0) && (mode <= 3))  mode += 4;        // Reached the hopper approach point, now beginning approach for that hopper
    else if ((mode < 0) || (mode > 9))  {  Serial.print("Something went wrong with modes. It thinks its at mode = "); Serial.println(mode);}
    return mode;
}



void disposal_servo(int move_count, int* column_priority, int* column_ball_count)
{
  int column;
  float angle;
  myServoGate.write(82);
  delay(500);
  
  column = connect_4();
  
  Serial.print("Dropping ball in column: ");
  Serial.println(column);
  
  switch(column){
    case('0'): {
      angle = 133;
      break;}
    case('1'): {
      angle = 110;
      break;}
    case('2'):{
      angle = 90;
      break;}
    case('3'): {
      angle = 65;
      break;}
    case('4'): {
      angle = 46;
      break;}
    case('5'): {
      angle = 23;
      break;}
    case('6'): {
      angle = 0;
      break;}
  }
 
   // NOTE: these angles were drawn from experimental testing due to imperfections in the design of the disposal system
                                    
  Serial.print("Disposal servo turning to angle: ");
  Serial.println(angle);

  myServoGate.write(58);                      // Partially open the gate to allow the disposal servo to move
  delay(1000);
  myDisposalServo.write(angle);               // Turn the disposal servo to the chosen column
  delay(1000);
  myServoGate.write(0);                       // Fully open the gate
  delay(1500);                                // Wait for the Servo motor to adjust and for us to drop the ball
  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, HIGH);
  digitalWrite(enablePin, HIGH);
  delay(200);
  digitalWrite(enablePin, LOW);
  delay(4000);                                // Drive into game board - wait for ball to trickle out
  
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
  digitalWrite(enablePin, HIGH);
  delay(200);
  digitalWrite(enablePin, LOW);
  digitalWrite(leftPin, HIGH);
  digitalWrite(rightPin, HIGH);

  myDisposalServo.write(65);          // After its done, move it back to the middle column
  column_ball_count[column]++;        // Modify the ball count in the game board
  delay(1000);
  myServoGate.write(82);                      // Close gate

  // Now gate is closed and disposal servo is in middle column and ball has been dropped off
  // Good to end disposal section
  return; 
}



int connect_4(){
    while(full_flag[7] != 1)             
    {
        LOOP:

        checkFF = 0;            // Checks to see if the board is full
        for (int i = 0; i < 7; i++)
        {    
            if(full_flag[i] > 0)
            {    checkFF++;}
        }
        if (checkFF == 7)
        {
            full_flag[7] = 1;
            goto END;
        }
        
        if ((AI_on == 0) || (turn == 0))    // P1's turn, or P2 if AI off 
        {
            Serial.print("Player ");
            Serial.print(turn + 1);
            Serial.println("'s turn");
            Serial.println("Drop the ball in which column? "); 
            
            while (1)
            {
              if (Serial.peek() != -1){
                inputcolumn = (int)(Serial.read() - (int)'0');
                break;}
            }
               
            if (inputcolumn == 7)          // Skip turn
            {
              turn = turn ^ 1;
              continue;
            }
                    
            if ((inputcolumn >= 6) && (inputcolumn <= 0)) 
            {    
                Serial.print(inputcolumn);
                Serial.println(" is not a valid column");
                goto LOOP;
            }
        }

       else 
        {    
            int moves_ahead = 3;
            Serial.print("Player ");
            Serial.print(turn + 1);
            Serial.println("'s turn");
            inputcolumn = connect_sum_ai(array, player_two_colour, player_one_colour, 0, moves_ahead, full_flag, row_horizontal_connects, column_vertical_connects, diagonal_line_connects);
            Serial.print("I DECIDED TO GO HERE:"); 
            Serial.println(inputcolumn);
            return inputcolumn;
        }

        Serial.print("Full flag state: ");
        for (int m = 0; m < 8; m++)
        {    Serial.print(full_flag[m] );}
        Serial.println();
 
        result = check_move(array, inputcolumn);
 
         
        // Handles the case in which the move is invalid 
        if (result == -1) 
        {
            Serial.println("Column is full. Try another move");
            full_flag[inputcolumn] = 1;
            goto LOOP;
        } 

        // Deals with points obtained from the move that just occured
        else
        {
            row = result; 
            if (turn == 0) 
            {        
                array[row][inputcolumn] = player_one_colour;
                if (row == 5) full_flag[inputcolumn] = 1;        
                temp = 4 * (horizontal_connect(array, inputcolumn, row, player_one_colour, row_horizontal_connects, -1) + vertical_connect(array, inputcolumn, row, player_one_colour, column_vertical_connects, -1) + diagonal_connect(array, inputcolumn, row, player_one_colour, diagonal_line_connects, -1));
                player_1_score += temp + 1;
        
                if (temp > 0)
                {    player_2_score -= 2 * (temp/4);}
            }
 
            else  
            {    array[row][inputcolumn] = player_two_colour;
                temp = 4 * (horizontal_connect(array, inputcolumn, row, player_two_colour, row_horizontal_connects, -1) + vertical_connect(array, inputcolumn, row, player_two_colour, column_vertical_connects, -1) + diagonal_connect(array, inputcolumn, row, player_two_colour, diagonal_line_connects, -1)); 
                player_2_score += temp + 1;
        
                if (temp > 0)
                {    player_1_score -= 2 * (temp/4);}
            } 
                 
        } 

        // To account for the point(s) case if only 1 point
        if (player_1_score != 1) 
        {  Serial.print("Player 1 has ");
           Serial.print(player_1_score);
           Serial.println(" points");
        }
        else 
        {  Serial.print("Player 1 has ");
           Serial.print(player_1_score);
           Serial.println(" point");
        }

       if (player_2_score != 1) 
        {  Serial.print("Player 2 has ");
           Serial.print(player_1_score);
           Serial.println(" points");
        }
        else 
        {  Serial.print("Player 2 has ");
           Serial.print(player_2_score);
           Serial.println(" point");
        }
   
        // Checking to see if the entire game board is full
        displayboard(array, full_flag);
        if (full_flag[7] > 0)    goto END;     
        turn = turn ^ 1;     
    } 
 
    END: 
    Serial.println("The game board is full. The game is over."); 
 
    if (player_1_score > player_2_score) 
    {    Serial.print("Player 1 wins ");
         Serial.print(player_1_score);
         Serial.print(" - ");
         Serial.println(player_2_score);
    } 
     
    else if (player_1_score == player_2_score) 
    {    Serial.print("Both players tied with ");
         Serial.print(player_1_score);
         Serial.println(" points");
    } 
 
    else 
    {    Serial.print("Player 2 wins ");
         Serial.print(player_2_score);
         Serial.print(" - ");
         Serial.println(player_1_score);
    } 

    for (int i = 0; i < 6; i++)
    {    free(array[i]);}
    free(array);
    free(full_flag);
 
    while(1) {}
    return 0;
} 
 
int check_move(char** array, int inputcolumn) 
{ 
    int row = 0; 
    for (row = 0; row < 6; row++)
    {
        if (array[row][inputcolumn] == ' ')
        {    break;}
    }

    if (array[5][inputcolumn] != ' ')
    {    return -1;} 
    return row; 
} 

 
// In the following three cases, moves_ahead acts to control print statements (only final iteration), and as a flag for the row/col/line connects arrays 
int horizontal_connect(char** array, int inputcolumn, int row, char colour, int* row_horizontal_connects, int moves_ahead)
{
    int count = 0;

    for (int i = inputcolumn; i < 7; i++)
    {    
        if (array[row][i] != colour)
        {    break;}

        count++;
    }

    for (int i = inputcolumn - 1; i >= 0; i--)
    {    
        if (array[row][i] != colour)
        {    break;}

        count++;
    }

    if ((count == 4) && (moves_ahead == -1))
    {    row_horizontal_connects[row] = 1;}


    // To see if there was already a connect in that row
    if (row_horizontal_connects[row] == 0)
    {    return (count >= 4);}
    return (count == 4);
} 


int vertical_connect(char** array, int inputcolumn, int row, char colour, int* column_vertical_connects, int moves_ahead)
{
    int count = 0;

    for (int i = row; i < 6; i++)
    {    
        if (array[i][inputcolumn] != colour)
        {    break;}

        count++;
    }

    for (int i = row - 1; i >= 0; i--)
    {    
        if (array[i][inputcolumn] != colour)
        {    break;}

        count++;
    }

    if ((count == 4) && (moves_ahead == -1))
    {    column_vertical_connects[inputcolumn] = 1;}

    // To see if there was already a connect in that row
    if (column_vertical_connects[inputcolumn] == 0)
    {    return (count >= 4);}
    return (count == 4);
}


int diagonal_connect(char** array, int inputcolumn, int row, char colour, int* diagonal_line_connects, int moves_ahead)
{
    int count = 0;
    int count1 = 0;
    int count2 = 0;
    int check1 = 0;
    int check2 = 0;

    for (int i = row, j = inputcolumn; (i < 6)&&(j < 7); i++, j++)
    {    
        if (array[i][j] != colour)
        {    break;}

        count1++;
    }

    for (int i = row, j = inputcolumn; (i < 6)&&(j >= 0); i++, j--)
    {    
        if (array[i][j] != colour)
        {    break;}

        count2++;
    }

    for (int i = row - 1, j = inputcolumn + 1; (i >= 0)&&(j < 7); i--, j++)
    {    
        if (array[i][j] != colour)
        {    break;}

        count2++;
    }

    for (int i = row - 1, j = inputcolumn - 1; (i >= 0)&&(j >= 0); i--, j--)
    {    
        if (array[i][j] != colour)
        {    break;}

        count1++;
    }

    // For count1, the difference of row - column is the same for all numbers in an up-right diagonal
    // For count2, the sum of row + column is the same for all numbers in an up-left diagonal

    if (count1 >= 4)
    {    
        for (int i = 0; i < 6; i++)
        {    
            if (diagonal_line_connects[i] == (row - inputcolumn))
            {    break;}

            if (diagonal_line_connects[i] == -10)
            {    if (moves_ahead == -1)
                {    diagonal_line_connects[i] = (row - inputcolumn);}
                check1 = (count1 >= 4);
                break;}
        }
    }

    if (count2 >= 4)
    {
        for (int i = 6; i < 12; i++)
        {    
            if (diagonal_line_connects[i] == (row + inputcolumn))
            {    break;}

            if (diagonal_line_connects[i] == -10)
            {    if (moves_ahead == -1)
                {    diagonal_line_connects[i] = (row + inputcolumn);}
                check2 = (count2 >= 4);
                break;}
        }
    }

    count = check1 + check2;
    return count;
} 
 
 
int displayboard(char** array, int* full_flag) 
{
    int count = 0; 
    for (int i = 5; i >= 0; i--) 
    {
        Serial.print(i); 
        for (int j = 0; j < 8; j++) 
        { 
            Serial.print("| ");
            Serial.print(array[i][j] );
            if ((i == 5) && ((array[i][j] == 'b') || (array[i][j] == 'w')))
            {    full_flag[j] = 1;
                count++;} 
        } 
        Serial.println(); 
    } 
    Serial.println("   0  1  2  3  4  5  6");

    if (count == 7)
    {    full_flag[7] = 1;
        return 1;}
    return 0; 
}


int connect_sum_ai(char** array, char AI_colour, char opponent_colour, int moves_ahead, int total_moves, int* full_flag, int* row_horizontal_connects, int* column_vertical_connects, int* diagonal_line_connects)
{
    int points[7] = {0,0,0,0,0,0,0};
    int maximum = -20; 
    int max_col = 0;
    char colour;
    int factor = total_moves - moves_ahead + 1;

    if (moves_ahead % 2 == 0) colour = AI_colour;
    else colour = opponent_colour;

    for (int col = 0; col < 7; col++)
    {
        int row = 0;
        if (array[5][col] != ' ') continue;

        for (row = 0; row < 6; row++)
        {
            if (array[row][col] == ' ')
            {    break;}            
        }

        array[row][col] = colour;
        points[col] += factor*horizontal_connect(array, col, row, colour, row_horizontal_connects, moves_ahead);
        points[col] += factor*vertical_connect(array, col, row, colour, column_vertical_connects, moves_ahead);
        points[col] += factor*diagonal_connect(array, col, row, colour, diagonal_line_connects, moves_ahead);

        if (moves_ahead < total_moves)            
        {    points[col] -= connect_sum_ai(array, AI_colour, opponent_colour, moves_ahead + 1, total_moves, full_flag, row_horizontal_connects, column_vertical_connects, diagonal_line_connects);}
        
        array[row][col] = ' ';
    }


    if (moves_ahead == 0) 
    {    Serial.print("points are: ");
        for (int i = 0; i < 7; i++) {Serial.print(points[i]);}
        Serial.println();
    }

    
    for (int col = 0; col < 7; col++)
    {
        if (full_flag[col] != 0)
        {    continue;}

        else            
        {
            if (points[col] > maximum)
            {    maximum = points[col];
                max_col = col;}

            else if (points[col] == maximum)        // In the case of a tie, check for which column is closest to the center and put it there
            {    
                if ((3-col)*(3-col) < (3-max_col)*(3-max_col))    // Check distance to middle column - smaller distance gets priority
                {    max_col = col;}
                else if ((3-col)*(3-col) == (3-max_col)*(3-max_col))
                {    if (col > 3) max_col = col;}
            }
        }
    }    

    if (moves_ahead > 0) 
    {    return maximum;}
    else 
    {    if (maximum > 0) 
         {  Serial.print("max points: ");
            Serial.print(maximum);
            Serial.print("col: ");
            Serial.println(max_col);
         }
        return max_col;
    }
}
