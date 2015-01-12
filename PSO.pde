


class particle  // create a new user defined structure called particle
{
  float[] pos;
  float[] vel;
  float[] Bpos;
  float fitness;
  float PARbestFitness; 
  color c;
  
  particle(float[] pos_, float[] vel_, float[] Bpos_, float fitness_, float PARbestFitness_, color c_) {
  pos = pos_;
  vel = vel_;
  Bpos = Bpos_;
  fitness = fitness_;
  PARbestFitness = PARbestFitness_;  
  c = c_;
  }

  /*void plot() {
    background(0); 
    for (int i = 0; i < swarn.length; i ++ ) { // Initialize each Car using a for loop.
      x= map(swarn[i].pos[1], minKp, maxKp, 0, width);
      y= map(swarn[i].pos[2], minKi, maxKi, 0, height);
      ellipse(x, y, 5, 5);
   
    }
  //print(x);print(" ");println(y);
  }*/

}

class space //Defines the space where the particle can move
{
  float minR;
  float maxR;    

  space(float minR_, float maxR_){
   minR = minR_;
   maxR =  maxR_;

  }
  
};

