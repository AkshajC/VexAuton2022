#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
digital_out frontMogo = digital_out(Brain.ThreeWirePort.A);
digital_out backMogo1 = digital_out(Brain.ThreeWirePort.B);
digital_out backMogo2 = digital_out(Brain.ThreeWirePort.D);
motor liftMotor = motor(PORT8, ratio18_1, true);
motor ringIntake = motor(PORT5, ratio18_1, false);
motor rightBackMotor = motor(PORT12, ratio18_1, false);
motor rightMiddleMotor = motor(PORT10, ratio18_1, true);
motor rightFrontMotor = motor(PORT21, ratio18_1, true);
motor leftMiddleMotor = motor(PORT16, ratio18_1, false);
motor leftBackMotor = motor(PORT18, ratio18_1, true);
motor leftFrontMotor = motor(PORT1, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}