package frc.robot.auto.sweetturn;

public class Constants
{

	// SWEET PROFILES


	/* Instructions for calibrating Sweet turn.
	* Goal: to calibrate turn before any profile uses it. Tuning autonomous profile should be angles only.
	*  Steps:
	* 1.)  Create turn only autonomous profile.
	* 2.)  Start with 90 Degree turn.
	* 3.)  Now test 5, 15, 45, 60,360 degree turns
	* 4.) test more if desired. Goal is to see a various of ranges.
	* 5.) I would also test if overshoot correction works. To do this cal the "SWEET_TURN_RAMP_DOWN_START_OFFSET_X" artificially low. This will force overshoot.
	*/


	/* ******************************************** SWEET TURN PROFILE ******************************************************************
	*                                                                    |<   RAMP_DOWN_START_OFFSET(YawRate) >|
	*
	*                                                                    |                                     |
	*                                 _______MAX_PCT(TotalTravel)________
	*  P                              /                                  \                                     |
	*  C                             /                                    \
	*  T                            /                                      \                                   |
	*                              /                                        T
	*                             E                                          U                                 |
	*                            T                                            R
	*                           A                                              N                               |
	*                          R                                                \
	*                         /                                                  D                             |
	*                        P                                                    O
	*                       U                                                      W                           |
	*                      /                                                        N
	*                     N                                                          \                         |
	*                    R                                                            R
	*                   U                                                              A                       |
	*                  T                                                                T
	*                 /                                                                  E                     |
	*                /                                                                    \                         Precise Turn speed should be the lowest pct
	*               /                                                                      \___PERCISE_TURN____|    command that will allow the robot to turn from
	*              /                                                                                           |    stop
	*             /                                                                                            |
	*            X                                                                                             X
	*    Starting Point                                                                                   target Point
	*                                                                                          Target Point can only be reached when
	*                                                                                     Yaw rate is less than SWEET_TURN_MAX_EXIT_VELOCITY
	*                         ANGLE
	*
	*
	*/


	/*
	 * SWEET_TURN_RAMP_UP_RATE is setup to reduce initial jerk when starting turn.
	 * It is measured in pct per loop. My initial value(.11) is so high it will only
	 * impact higher speed turns.
	 */
	public static final double SWEET_TURN_RAMP_UP_RATE = 0.11f; // pct/loop(20ms)

	/*
	 * SWEET_TURN_MAX_PCT_TOTAL_CHANGE_X and SWEET_TURN_MAX_PCT_X are paired. Turns
	 * that go farther can have a higher max speed. Since they have more time to
	 * slow down.
	 *
	 * Think of SWEET_TURN_MAX_PCT_TOTAL_CHANGE_X x axis and SWEET_TURN_MAX_PCT_X
	 * like the y axis of Func Then linear doubleerpolation is used to take the
	 * output = Func(Total Distance of turn)
	 */
	public static final double SWEET_TURN_MAX_PCT_TOTAL_CHANGE_0 = 5; // Degree
	public static final double SWEET_TURN_MAX_PCT_TOTAL_CHANGE_1 = 45; // Degree
	public static final double SWEET_TURN_MAX_PCT_TOTAL_CHANGE_2 = 90; // Degree
	public static final double SWEET_TURN_MAX_PCT_0 = 0.3; // pct
	public static final double SWEET_TURN_MAX_PCT_1 = 0.8; // pct
	public static final double SWEET_TURN_MAX_PCT_2 = 1.0; // pct

	/*
	 * SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_X and
	 * SWEET_TURN_RAMP_DOWN_START_OFFSET_X are paired The faster you are going the
	 * longer it will take to slow down to a controllable speed. The faster you are
	 * moving the sooner the ramp down to the precise turn speed needs to occur. For
	 * example if your yaw rate is 220 degrees per second you need more distance to
	 * stop then if your yaw rate is 50 degrees per second. This is the parameter
	 * with the biggest impact on overshoot.
	 *
	 * Think of SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_X x axis and
	 * SWEET_TURN_RAMP_DOWN_START_OFFSET_X like the y axis of Func Then linear
	 * doubleerpolation is used to take the output = Func(Yaw Rate)
	 */
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_0 = 50; // degrees/Second
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_1 = 180; // degrees/Second
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_2 = 220; // degrees/Second
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_0 = 15; // Degree
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_1 = 35; // Degree
	public static final double SWEET_TURN_RAMP_DOWN_START_OFFSET_2 = 75; // Degree

	/*
	 * SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_X and SWEET_TURN_RAMP_DOWN_RATE_X
	 * are paired When ramping down the less distance you have remain the faster you
	 * need to ramp down to the precise turn speed. Based on testing the ramps take
	 * any where from a single 20ms loop to 5 20ms loops. A step might be
	 * acceptable.
	 *
	 * Think of SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_X x axis and
	 * SWEET_TURN_RAMP_DOWN_RATE_X like the y axis of Func Then linear
	 * doubleerpolation is used to take the output = Func(Angle Remaining at start
	 * of ramp down)
	 */
	public static final double SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_0 = 10; // Degree
	public static final double SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_1 = 40; // Degree
	public static final double SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_2 = 60; // Degree
	public static final double SWEET_TURN_RAMP_DOWN_RATE_0 = 1.0; // pct/loop(20ms)
	public static final double SWEET_TURN_RAMP_DOWN_RATE_1 = 0.6; // pct/loop(20ms)
	public static final double SWEET_TURN_RAMP_DOWN_RATE_2 = 0.2; // pct/loop(20ms)

	/*
	 * SWEET_TURN_PERCISE_TURN_PCT is the minimum pct output to the motors that will
	 * still turn the robot. The goal is that if we step to zero pct from
	 * SWEET_TURN_PERCISE_TURN_PCT the robot will stop at the same location or
	 * within the target.
	 */
	public static final double SWEET_TURN_PERCISE_TURN_PCT = 0.055;

	/*
	 * SWEET_TURN_MAX_EXIT_VELOCITY is the maximum yaw rate in degrees per second
	 * that be present to allow the sweet turn to complete. If the yaw rate is
	 * greater an overshoot is occurring and a correction step will take place.
	 * Originally set to 75 degrees per second. If too low unneeded corrections will
	 * occur and sweet turn can get stuck in a loop
	 */
	public static final double SWEET_TURN_MAX_EXIT_VELOCITY = 55.0; // degrees/Second

	/*
	 * SWEET_TURN_ITERATE_MAX number of corrections allowed if an overshoot occurs.
	 */
	public static final double SWEET_TURN_ITERATE_MAX = 3;

	public static final Point2D SweetTurnMaxPct[] =
	{ new Point2D(SWEET_TURN_MAX_PCT_TOTAL_CHANGE_0, SWEET_TURN_MAX_PCT_0),
			new Point2D(SWEET_TURN_MAX_PCT_TOTAL_CHANGE_1, SWEET_TURN_MAX_PCT_1),
			new Point2D(SWEET_TURN_MAX_PCT_TOTAL_CHANGE_2, SWEET_TURN_MAX_PCT_2) };
	public static final Point2D SweetTurnRampDownStartOffSet[] =
	{ new Point2D(SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_0, SWEET_TURN_RAMP_DOWN_START_OFFSET_0),
			new Point2D(SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_1, SWEET_TURN_RAMP_DOWN_START_OFFSET_1),
			new Point2D(SWEET_TURN_RAMP_DOWN_START_OFFSET_YAW_RATE_2, SWEET_TURN_RAMP_DOWN_START_OFFSET_2) };
	public static final Point2D SweetTurnRampDownRate[] =
	{ new Point2D(SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_0, SWEET_TURN_RAMP_DOWN_RATE_0),
			new Point2D(SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_1, SWEET_TURN_RAMP_DOWN_RATE_1),
			new Point2D(SWEET_TURN_RAMP_DOWN_RATE_ANGLEREMAINING_2, SWEET_TURN_RAMP_DOWN_RATE_2) };
}