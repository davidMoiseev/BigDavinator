/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Declare new Talon
 * Declare new Joystick
 * Declare new String Builder
 * IN ROBOT INIT:
 * Chose the Encoder Type
 * Set Sensor to False
 * Set Voltage Perameters
 * Set Profile and FPID Controls
 * Set Acceleration
 * Set Cruise Velocity
 * IN TELEOP PERIODIC
 * Get Joystick Axis
 * Get Voltage
 * Add Output to String Builder
 * Add Motor Output to String Builder
 * Add Speed to String Builder
 * Get Robot Speed; Add to String Builder
 * IF (a Button) is Pressed, THEN:
 * Set Target Speed to the value of the Joystick Axis * 10
 * Change Talon Control Mode to Motion Magic
 * Set Target Speed
 * Add Error to String Builder
 * Add Close Loop Talon Error to String Builder
 * Add Target to String Builder
 * Add Target Speed to String Builder
 * ELSE:
 * Change Talon Control Mode to Percent V-Bus
 * Set Joystick
 * END IF/ELSE LOOP
 * Instrum.Process(talon, stringbuilder)
 */
public class MotionMagicTest {
        CANTalon _talon = new CANTalon(3);
        Joystick _joy = new Joystick(0);
        StringBuilder _sb = new StringBuilder();
        public void robotInit() {
        /* first choose the sensor */
        _talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        _talon.reverseSensor(false);
        // _talon.configEncoderCodesPerRev(XXX)
        // _talon.configPotentiometerTurns(XXX)
        /* set the peak and nominal outputs, 12V means full */
        _talon.configNominalOutputVoltage(+0.0f, -0.0f);
        _talon.configPeakOutputVoltage(+12.0f, -12.0f);
        /* set closed loop gains in slot0 */
        _talon.setProfile(0);
        _talon.setF(0);
        _talon.setP(0);
        _talon.setI(0);
        _talon.setD(0);
        _talon.setMotionMagicAcceleration(0);
        _talon.setMotionMagicCruiseVelocity(0);
        }
        /**
        * This function is called periodically during operator control
        */
        public void teleopPeriodic() {
        /* get gamepad axis */
        double leftYstick = _joy.getAxis(AxisType.kY);
        double motorOutput = _talon.getOutputVoltage() / _talon.getBusVoltage();
        /* prepare line to print */
        _sb.append("\tout:");
        _sb.append(motorOutput);
        _sb.append("\tspd:");
        _sb.append(_talon.getSpeed());
        if (_joy.getRawButton(1)) {
        /* Speed mode */
        double targetSpeed = leftYstick
        * 10.0; /* 10 Rotations in either direction */
        _talon.changeControlMode(TalonControlMode.MotionMagic);
        _talon.set(targetSpeed); /* 1500 RPM in either direction */
        /* append more signals to print when in speed mode. */
        _sb.append("\terr:");
        _sb.append(_talon.getClosedLoopError());
        _sb.append("\ttrg:");
        _sb.append(targetSpeed);
    } else {
        /* Percent voltage mode */
        _talon.changeControlMode(TalonControlMode.PercentVbus);
        _talon.set(leftYstick);
    }
        /* instrumentation */
        Instrum.Process(_talon, _sb);
    }
}