/*
 * Macros to set:
 * PIN_STEP - Pin on Arduino to which the step pin is connected.
 * PIN_DIRECTION - Pin on Arduino on which direction pin is connected.
 * THREAD_ROD_PITCH - The pitch in um for the threaded rod.
 * STEPS_PER_REVOLUTION - Number of steps for 1 revolution. This is
 *                      dependant on the driver used. For Easy Driver - 1600
 * BASE_ARM_LENGTH - Length of one of the similar arms in the isosceles triangle.
 * ANGLE_PER_SECOND - Angle to be traversed by the earth's rotation in 1 sec.
 * START_ANGLE - Angle between the similar sides when tracking is started.
 * CALCULATION_INTERVAL - Intervals between calculation.
 */

#include <math.h>
#include <AccelStepper.h> /* Stepper Library */
#include <SM.h> /* Finite State Machine */

/* If #define Debug exists, debug_print will work. */
/* Comment out the line below to stop debug output */
#define DEBUG
#ifdef DEBUG
#define debug_print(m) Serial.print(m)
#else
#define debug_print(m)
#endif

/* Finite State Machine - Globals */
unsigned int stepper_state;
#define STEPPER_STATE_INIT 0
#define STEPPER_STATE_SLEEP 1
#define STEPPER_STATE_RUN 2
SM Stepper_FSM(StepperInit);

/* Stepper motor */
#define PIN_STEP 8
#define PIN_DIRECTION 9
#define THREAD_ROD_PITCH 1000 /* um */
#define STEPS_PER_REVOLUTION 1600 /* steps/revolution */
double steps_per_um = (double)STEPS_PER_REVOLUTION/THREAD_ROD_PITCH;
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

/* Tracker */
#define BASE_ARM_LENGTH 400 /* mm */
#define ANGLE_PER_SECOND 0.0041780 /* degrees/sec */
#define RAD(deg) (deg*PI/180)
#define START_ANGLE 0 /* Angle at which we start */
#define CALCULATION_INTERVAL 10 /* seconds */

unsigned int tracker_state;
#define TRACKER_STATE_INIT 0
#define TRACKER_STATE_SLEEP 1
#define TRACKER_STATE_TRACK 2
#define TRACKER_STATE_REVERSE 3
#define TRACKER_STATE_INIT_FORWARD 4
SM Tracker_FSM(TrackerInit);
unsigned int tracker_start_time=0;
double tracker_existing_rod_length = 2 * BASE_ARM_LENGTH *
                                sin(START_ANGLE/(2*3.14156));

/*
 * Calculate what the length of the rod should be at that
 * particular time
 *
 * args : time in seconds.
 * returned: distance micrometer
 */
unsigned int calculate_length_of_rod(int time)
{
  double angle = START_ANGLE + (time*ANGLE_PER_SECOND);

  //debug_print("time "); debug_print(time);
  //debug_print(" angle "); mprint_double(angle,10000);

  return 2 * BASE_ARM_LENGTH * sin(RAD(angle/2)) * 1000;
}

/*
 * Called to rotate the stepper motor to move the rod
 * by the required distance.
 *
 * args: length is in micrometers
 * returned: actual length moved in micrometers
 */
double move_arm(unsigned int length)
{
  static unsigned long total;
  unsigned int steps_required;
  float speed;

  total += length;

  stepper_state = STEPPER_STATE_RUN;
  steps_required = length * steps_per_um;
  speed = (float) (stepper.distanceToGo()+ steps_required)/CALCULATION_INTERVAL;

  debug_print("length: "); debug_print(length); debug_print("\n");
  debug_print("Number of steps: "); debug_print(steps_required); debug_print("\n");
  debug_print("speed: "); debug_print(speed); debug_print(" steps/second\n");

  stepper.move(steps_required);
  stepper.setSpeed(speed);

  return (double)steps_required/steps_per_um;
}

void setup(void)
{
  Serial.begin(9600);
  stepper_state = STEPPER_STATE_INIT;
}

void loop(void)
{
  EXEC(Tracker_FSM);
  EXEC(Stepper_FSM);
  //stepper.runSpeed();
}

/* Stepper States */
State StepperInit()
{
  debug_print("Stepper Init\n");

  stepper_state = STEPPER_STATE_INIT;
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);

  Stepper_FSM.Set(StepperSleepState);
}

State StepperSleepState()
{
//  debug_print("Stepper Sleep\n");
  switch(stepper_state)
  {
    case STEPPER_STATE_INIT:
      Stepper_FSM.Set(StepperInit);
      break;
    case STEPPER_STATE_RUN:
      Stepper_FSM.Set(StepperRun);
      break;
  }
}

State StepperRun()
{
  //debug_print("Stepper Run\n");
  stepper.runSpeed();
  if(stepper.distanceToGo() == 0){
    stepper_state = STEPPER_STATE_SLEEP;
    Stepper_FSM.Set(StepperSleepState);
  }
}

/* Tracker States */
State TrackerInit()
{
  debug_print("Tracker Init\n");
  tracker_state = TRACKER_STATE_TRACK;
  Tracker_FSM.Set(TrackerSleepState);
}

State TrackerSleepState()
{
//  debug_print("Tracker Sleep\n");

  switch(tracker_state)
  {
    case TRACKER_STATE_INIT:
      Tracker_FSM.Set(TrackerInit);
      break;
    case TRACKER_STATE_TRACK:
      Tracker_FSM.Set(TrackerTrackInit);
      break;
  }
}

State TrackerTrackInit()
{
  debug_print("Tracker Track Init\n");
  tracker_existing_rod_length = 2 * BASE_ARM_LENGTH *
                                sin(START_ANGLE/(2*3.14156));
  tracker_start_time = millis();
  Tracker_FSM.Set(TrackerTrack);
}

State TrackerTrack()
{
  debug_print("Tracker Track\n");
  unsigned int time_run;
  double length, length_to_move;

  time_run = (millis() - tracker_start_time)/1000;
  //Add next interval
  time_run += CALCULATION_INTERVAL;
  length = calculate_length_of_rod(time_run);
  length_to_move = length - tracker_existing_rod_length;
  tracker_existing_rod_length += move_arm(length_to_move);

  Tracker_FSM.Set(TrackerTrackWait);
}

State TrackerTrackWait()
{
  //debug_print("Tracker Track Wait\n");
  if (Tracker_FSM.Timeout(CALCULATION_INTERVAL*1000))
    Tracker_FSM.Set(TrackerTrack);
}

