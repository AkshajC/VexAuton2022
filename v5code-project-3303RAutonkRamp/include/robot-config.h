using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern digital_out frontMogo;
extern digital_out backMogo1;
extern digital_out backMogo2;
extern motor liftMotor;
extern motor ringIntake;
extern motor rightBackMotor;
extern motor rightMiddleMotor;
extern motor rightFrontMotor;
extern motor leftMiddleMotor;
extern motor leftBackMotor;
extern motor leftFrontMotor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );