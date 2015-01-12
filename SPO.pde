//This script must run as a timed action. Target function (ISTE) is calculate in the main loop and resetted here

/*
For the moment just calculate ISTE, in future define some criteria to ensure the robot doesn't fall
 even in case of wrong parameters, e.g. if dISTE > than Max, reset the particle to its best known pos
 and PID to overall best known pos?
 
 https://github.com/kiuz/Arduino-Statistic-Library
 Need to add the measure using the Statistic library and check if stdev goes big
 As an alternative we can use the derivative ISTE2-ISTE1
 
 */

boolean debugSPO = false;
String SPACER = " ";
String Note = "";
String LastEventSPO ="";
//char cbuffer[10];
//Init code goes in the main body of the sketch
int numParticles = 100;
int particleNumber = 0;
float ISTE=0;
float maxInteractions = 100;
float bestParticle = 0;
float bestIteraction = 0;
float SPOiteraction = 0;
//int feedbackTime = 3000; //feedback time in milliseconds. Maybe we can make it dynamic, first short time, after long time
float bestGlobalFitness = 9999;
//float maxIncreaseRate = 1; //condition to stop the particle test in before the feedbackTime if the error rate increase out of control
//Define a percentage arounf the known stable values
int spread = 10;
/*float    minKp =  configuration.anglePIDConKp * (1 - spread/100), 
 maxKp =  configuration.anglePIDConKp * (1 + spread/100),
 minKi =  configuration.anglePIDConKi * (1 - spread/100),
 maxKi =  configuration.anglePIDConKi * (1 + spread/100),
 minKd =  configuration.anglePIDConKd * (1 - spread/100), 
 maxKd =  configuration.anglePIDConKd * (1 + spread/100);
*/
//Search space for test
float minKp=-50, maxKp=50, minKi=-50, maxKi=50, minKd=-50, maxKd=50;
int widht = 640;
int height = 360;
String LOG ="N/A";
//float minKp = -20, maxKp = 20, minKi = -20, maxKi = 20, minKd = -50, maxKd =50;
//Need to be smarter. Define a function based on: a) domain, d/dt of ISTE 
float maxVel = 15, startVel;//min(minKp, minKi)/10;

float[] bestGlobalPosition = new float[3];


float curVal = 9999;
  float bestVal = 9999;
 //float[] = v[2];
  float[] pos = new float[3];
  float[] vel = new float[3];
  float[] Bpos = new float[3];

particle[] swarn = new particle[numParticles]; 
space[] domain = new space[3];

float ISTEF(float x, float y, float z)
{
  //x, y, z are for testing, when you us an analytical function instead of ISTE
  //z = x * exp( -(x^2 + y^2) ) ;The function has a known minimum value of z = -0.4288819 at x = -0.7071068 and y = 0.0.
  return  x * exp( -(x*x + y*y) ); 
  //return -1/(x*x+y*y+1);
  //return x+y+z;
}

void setup() {
 background(0);  
 
 size(widht, height);
 smooth(); 
 //fill(255, 126);

  //randomSeed(analogRead(0));
  //Initialize the swarn positions and velocites
 for (int i = 0; i < swarn.length; i ++ ) { // Initialize each Car using a for loop.
    float[] posT = {random(minKp, maxKp),random(minKi, maxKi),random(minKd, maxKd)};
    float[] velT = {random(minKp, maxKp),random(minKi, maxKi),random(minKd, maxKd)};
    float[] BposT = {random(minKp, maxKp),random(minKi, maxKi),random(minKd, maxKd)};
    color cT = color(random(1, 255), random(1, 255), random(1, 255));    
    swarn[i] = new particle(posT, velT, BposT,ISTEF(1,1,1),ISTEF(BposT[0],BposT[1],BposT[2]), cT);     
  }
  
 for (int i = 0; i < domain.length; i ++ ) { // Initialize each Car using a for loop.
   domain[i] = new space(0,0); 
 } 
  
  domain[0].minR = minKp;
  domain[0].maxR = maxKp;
  domain[1].minR = minKi;
  domain[1].maxR = maxKi;  
  domain[2].minR = minKd;
  domain[2].maxR = maxKd;
 
/* for (int i = 0; i < numParticles; i++){
    LOG = str(i) + '¦' + str(swarn[i].pos[0]) + ' '+ str(swarn[i].pos[1]) + '¦' + str(swarn[i].PARbestFitness);
    println(LOG); 
 }*/
 startVel = maxVel;
}

void draw(){
  float x, y;
  SPO();
    background(0); 
    for (int i = 0; i < swarn.length; i ++ ) { // Initialize each Car using a for loop.
      x= map(swarn[i].pos[1], minKp, maxKp, 0, width);
      y= map(swarn[i].pos[2], minKi, maxKi, 0, height);
      if (i == bestParticle) {
        fill(255,0,0);
        } 
        else {
          //fill(swarn[i].c);
          fill(255,255,255);
        }
      ellipse(x, y, 5, 5);
    }
    
    textSize(14);
    LOG= "I: " + int(SPOiteraction) + " MV: " +  String.format("%.3f", (maxVel));
    text(LOG, 10, 30); 
    
    LOG= "B: " + str(int(bestParticle)) + " I: " + str(int(bestIteraction)) + " F: " + String.format("%.3f", (bestGlobalFitness)) + " P: " + String.format("%.2f", (bestGlobalPosition[0])) + " " + String.format("%.3f", (bestGlobalPosition[1]));
    text(LOG, 10, 50); 
    

}


//------------------------------------------




//Call this as a scheduled task with feedbackTime as a trigger. The task is checked only if the varible AUTOTUNE is true. 
//SCMD AUTOTUNE 1
//------------------------------------------


void SPO(){

  //Particle data
  float w = 0.729; // inertia weight
  float c1 = 1.49445; // cognitive weight def 1.49445
  float c2 = 1.49445; // social weight
  float c1Min = 1, c1Max = 1.8;
  float c2Min = 1, c2Max = 1.8;
  float curVal = 9999;

  float r1, r2; // randomizers
  float newVel, newPos, mv; 
  
  curVal = ISTEF(swarn[particleNumber].pos[0], swarn[particleNumber].pos[1], swarn[particleNumber].pos[2]);
 
   //Need to check the target value of the current position
  if (curVal < swarn[particleNumber].PARbestFitness) {
    swarn[particleNumber].PARbestFitness = curVal;
    for (int j = 0; j < 3; j++){
      swarn[particleNumber].Bpos[j] = swarn[particleNumber].pos[j];
    };
  }
  if (curVal < bestGlobalFitness) {
    bestGlobalFitness = curVal;
    for (int j = 0; j < 3; j++){
      bestGlobalPosition[j] = swarn[particleNumber].pos[j];
    };
    //println("*********************NEW BEST FOUND*********************"); 
    bestParticle = particleNumber;
    bestIteraction = SPOiteraction;
  }

  for (int j = 0; j < 3; j++){
    r1 = random(0, 999);
    r2 = random(0, 999);
    r1 = r1/999;
    r2 = r2/999;
    c1= (c1Min-c1Max)*(SPOiteraction/maxInteractions)+c1Max;
    c2= (c2Max-c2Min)*(SPOiteraction/maxInteractions)+c2Min;

    newVel = w*swarn[particleNumber].vel[j] + c1*r1*(swarn[particleNumber].Bpos[j] - swarn[particleNumber].pos[j]) + c2*r2*(bestGlobalPosition[j] - swarn[particleNumber].pos[j]);
    swarn[particleNumber].vel[j] = newVel;   
       //Check if vel is allowed
    if (swarn[particleNumber].vel[j] > maxVel){
      swarn[particleNumber].vel[j] = maxVel;
      Note = Note + ("***Maxvel MAX*** ") + j; 
    }
    if (swarn[particleNumber].vel[j] < -maxVel){
      swarn[particleNumber].vel[j] = -maxVel;
      Note = Note + ("***Maxvel LOWER*** ") + j; 
    }

    //Now update the position
    newPos = swarn[particleNumber].pos[j] + swarn[particleNumber].vel[j];
    swarn[particleNumber].pos[j] = newPos;

    //Check if position is allowed
    if (swarn[particleNumber].pos[j] < domain[j].minR){
      swarn[particleNumber].pos[j] = random(domain[j].minR,domain[j].maxR); //
      Note = Note + ("***OOB LOWER*** ") + j; 
    }
    if (swarn[particleNumber].pos[j] > domain[j].maxR){
      swarn[particleNumber].pos[j] = random(domain[j].minR,domain[j].maxR);  //domain[j].maxR;
      Note = Note + ("***OOB MAX*** ") + j; 
    }

  }      
  
  if (debugSPO == true){
     println(LastEventSPO);
  }    


  //curVal = ISTEF(swarn[particleNumber].pos[0], swarn[particleNumber].pos[1], swarn[particleNumber].pos[2]);

  //swarn[particleNumber].plot(swarn[particleNumber].pos[0],swarn[particleNumber].pos[1]);
  //print(particleNumber);print(" ");print(curVal);   print(" ");print(swarn[particleNumber].pos[0]);  print(" "); println(swarn[particleNumber].pos[1]);

  //Reset ISTE and update the other global variables
  //ISTE=0;
  Note = "";
  particleNumber = particleNumber + 1;
  //If I have looped thorugh all the particles, the cycle starts again in a subsequent interaction
  
   if (particleNumber == numParticles) {
    SPOiteraction = SPOiteraction + 1;
    maxVel = 0.1 + (1-exp(-1+SPOiteraction/maxInteractions))*startVel;
    //maxVel = startVel * (1 - SPOiteraction/maxInteractions);
    particleNumber = 0;
    
  }

//Stop condittions: if I finish the number of cycles OR if the solution seems not to improve for some cycles
  if (SPOiteraction == maxInteractions || (SPOiteraction - bestIteraction) > 300){
    
   /* LastEventSPO = ("\n******************************************************************")+ SPACER + "\nInt: " + (int)bestIteraction + (" Particle: ") + (int)bestParticle + SPACER 
              +  "\nBest " + dtostrf(bestGlobalFitness, 10, 3, cbuffer) + SPACER + "Best Global: " + dtostrf(bestGlobalPosition[0], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[1], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[2], 10, 3, cbuffer)
              + ("\n******************************************************************"); 
   */
   if ((SPOiteraction - bestIteraction) > 300)
   {
    println("\nSoultion seems not to converge further... terminating");
   }
   
   if (debugSPO == true){println(LastEventSPO);}
  
   //AUTOTUNE = 0;
   SPOiteraction = 0;
   particleNumber = 0;
  }
}


