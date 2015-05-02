#define MIN_VALUE 0
#define MAX_VALUE 1000

#define RADIO_K 0.4

int PIDoutputToSpeed(int escs_speed[], int throttle, int roll, int pitch, int yaw) {
  if (throttle == 0) {
    roll  = 0;
    pitch = 0;
    yaw   = 0;
  }

  #ifdef DEBUG_NO_YAW_CONTROL
  yaw = 0;
  #endif

  escs_speed[0] = throttle + pitch + yaw;     //Y-axis (0h)
  escs_speed[1] = throttle + roll  - yaw;     //X-axis (3h)
  escs_speed[2] = throttle - pitch + yaw;     //Y-axis (6h)
  escs_speed[3] = throttle - roll  - yaw;     //X-axis (9h)
  
  limitBoundaries(escs_speed);
}

void limitBoundaries(int escs_speed[]) {
  // get min and max values
  int minv = escs_speed[0];
  int maxv = escs_speed[0];
  
  for(int i=1; i<4; i++) {
    if (escs_speed[i] < minv)
      minv = escs_speed[i];
    if (escs_speed[i] > maxv)
      maxv = escs_speed[i];
  }
  

  // get "proportional factor" if range is bigger than allowed
  float m = 1.0;
  if ((maxv - minv) > (MAX_VALUE - MIN_VALUE))
    m = (MAX_VALUE - MIN_VALUE) / (float)(maxv - minv);
    
    
  // get "shift value" after "proportional factor"
  int s = 0;
  if (maxv*m > MAX_VALUE)
    s = (MAX_VALUE - maxv*m);
  else if (minv*m < MIN_VALUE)
    s = (MIN_VALUE - minv*m);


  // correct
  for(int i=0; i<4; i++) {
    escs_speed[i] = escs_speed[i]*m + s;
  }  
}

