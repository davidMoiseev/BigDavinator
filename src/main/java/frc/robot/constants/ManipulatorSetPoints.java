/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * Add your docs here.
 */
public class ManipulatorSetPoints {

    public static final double WRIST_HEIGHT_LOW_CARGO = 27.5;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_LOW_CARGO = 36;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_LOW_CARGO_REVERSE = -36;//Degrees

    public static final double WRIST_HEIGHT_LOW_HATCH = 19;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_LOW_HATCH = 90;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_LOW_HATCH_REVERSE = -90;//Degrees

    public static final double WRIST_HEIGHT_MIDDLE_CARGO = 55.5;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO = 36;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO_REVERSE = -36;//Degrees

    public static final double WRIST_HEIGHT_MIDDLE_HATCH = 47;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH = 90;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH_REVERSE = -90;//Degrees

    public static final double WRIST_HEIGHT_HIGH_CARGO = 83.5;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_HIGH_CARGO = 36;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_HIGH_CARGO_REVERSE = -36;//Degrees

    public static final double WRIST_HEIGHT_HIGH_HATCH = 75;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_HIGH_HATCH = 90;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_HIGH_HATCH_REVERSE = -90;//Degrees

    public static final double WRIST_HEIGHT_CARGO_HOLD = 39.75;//Inches
    public static final double WRIST_ANGLE_PLACEMENT_CARGO_HOLD = 36;//Degrees
    public static final double WRIST_ANGLE_PLACEMENT_CARGO_HOLD_REVERSE = -36;//Degrees

    public static final double WRIST_HEIGHT_Ground = 4/*????It's a guess????*/;//Inches
    public static final double WRIST_ANGLE_PICKUP = 70;//Degrees
    public static final double WRIST_ANGLE_PICKUP_REVERSE = -70;//Degrees

    public static final double ELEVATOR_HEIGHT_CARRY = 3;//+3 Inches
    public static final double WRIST_ANGLE_CARRY = 10;//Degrees
    public static final double WRIST_ANGLE_CARRY_REVERSE = -10;//Degrees

    /* Lowest port hole (where the cargo is placed) center on spaceship is 27.5in. off the ground.
    Middle port hole (where the cargo is placed) center on spaceship is 55.5in. off the ground.
    Top port hole (where the cargo is placed) center on spaceship is 83.5in. off the ground.
    The center of the lowest hatch pannel spot on spaceship is 19in. off the ground.
    The center of the middle hatch pannel spot on spaceship is 47in. off the ground.
    The center of the top hatch pannel spot on spaceship is 75in. off the ground.
    
    Approx. 70 Deg.L (or -70 Deg.L) @ wrist to pick up cargo.
    Approx. 36 Deg.L (or -36 Deg.L) @ wrist to place cargo.
    90 Deg.L (or -90 Deg.L) @ wrist to place hatches.
    */

}
