#include "vex.h"
using namespace vex;
void autoarm(){
  hang1.setTimeout(2, seconds);                       
  hang1.spinToPosition(-280, degrees, 50, rpm, true); 
  if (hang1.isSpinning())
  {
    hang1.stop(brake);
  }
} 