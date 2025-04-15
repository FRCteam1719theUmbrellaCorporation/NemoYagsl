// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.AbsoluteEncoder;

/**
 * This controls the Coral Intake device
 * 
 * This device has 2 portions to it: the angle of the device, and the intake wheels
 * 
 * The intake angle is set depending on where the robot is intaking from (human station, floor) or outtaking
 * 
 * The wheels are also based on a current position, where they will intake at different speeds.
 * There is also a range sensor, which can allow us to detect half intake and other great things. this is so we can spit coral out and stuff like that
 * 
 * Overall, this subsystem is VITAL to our game plan, as it is our only intake
 */
public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  public static enum IntakePosition {
    FLOOR(CoralArmConstants.coral_floorintake_pos, CoralArmConstants.coral_intake_floor_speed), 
    DRIVING(CoralArmConstants.coral_floorintake_pos), 
    REEF(CoralArmConstants.coral_reef_l1, CoralArmConstants.coral_outtake_reef_speed), 
    HUMAN_STATION(CoralArmConstants.coral_floorintake_pos,CoralArmConstants.coral_intake_humanStation_speed);

    double angle; 
    double speed;
    int button;

    IntakePosition(double angle, double speed) {
      this.angle = angle;
      this.speed = speed;
    }

    IntakePosition(double angle) {
      this.angle = angle;
      speed = 0;
    }

  }
  private PIDController coralArmPIDController = new PIDController(CoralArmConstants.CoralArm_kP, 
                                                                    CoralArmConstants.CoralArm_kI, 
                                                                    CoralArmConstants.CoralArm_kD); 
                                                                    // TODO ADD CONSTANT 

  // HARDWARE
  private static SparkMax CORAL_ARM_TURNMOTOR;
  private static SparkMax CORAL_ARM_ANGLEMOTOR;
  private static AbsoluteEncoder CORAL_ARM_ENCODER;
  private static CANrange m_rangeSensor;

  private static double SETPOINTANGLE; // setpoint where the angle motor wants to go
  private static IntakePosition intakeMode; 

  public CoralIntakeSubsystem() {
    // defines hardware
    CORAL_ARM_TURNMOTOR = new SparkMax(CoralArmConstants.CORAL_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
    CORAL_ARM_ANGLEMOTOR = new SparkMax(CoralArmConstants.CORAL_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);
    CORAL_ARM_ENCODER = CORAL_ARM_ANGLEMOTOR.getAbsoluteEncoder();
    m_rangeSensor = new CANrange(CoralArmConstants.CORAL_ARM_RANGE_SENSOR);

    // updates the PID 
    coralArmPIDController.enableContinuousInput(0,1); 
    coralArmPIDController.setTolerance(5,10);

    // sets inital positions and settings
    SETPOINTANGLE = CoralArmConstants.coral_armdriving_pos;
    intakeMode = IntakePosition.DRIVING; 
  }

  /**
   * Turns the intake wheels at the specified speed
   * 
   * @param speed: speed specified
   * 
   */
  public void intakeSpinWheels(double speed){
    CORAL_ARM_TURNMOTOR.set(speed);
  }

  // stops the intake
  public void turnOffIntakeWheel(){
    CORAL_ARM_TURNMOTOR.stopMotor();
  }

  /**
   * Gets the position of the coral arm
   * 
   * @return CORAL_ARM_ENCODER.getPosition()
   */
  public double doubleMeasurement() {
    return CORAL_ARM_ENCODER.getPosition();
  }

  /**
   * sets the speed that the coral arm must move at
   * 
   * @param angle: the speed at which the arm should move
   */
  public void setRotation(double angle) {
    CORAL_ARM_ANGLEMOTOR.set(angle);
  }

  /**
   * Sets the arms move speed to 0
   * 
   */
  public void stopRotation() {
    CORAL_ARM_ANGLEMOTOR.set(0);
  }

  /**
   * Checks if the intake is on the floor
   * 
   * @return if the intake is on the floor
   */
  public boolean isIntaking() {
    return intakeMode == IntakePosition.FLOOR;
  }

  /**
   * Gets the current intake mode, as an enum
   * 
   * @return intake mode
   */
  public IntakePosition currentMode() {
    return intakeMode;
  }

  
  /**
   * @return Coral arm encoder
   */
  public AbsoluteEncoder getEncoder() {
    return CORAL_ARM_ENCODER;
  }

  /**
   * Current setpoint the arm is being moved to
   * 
   *  @return the current setpoint the arm is being moved to
   */
  public double getSetpoint() {
    return SETPOINTANGLE;
  }

  /**
   * PID for the arm
   * 
   *  @return the arm's PID controller
   */
  public PIDController getPID() {
    return coralArmPIDController;
  }  

  /**
   * Sets the arms setpoint. this will then cause the arm to move there
   * 
   *  @param point position from 0 - 1 that the intake will point to
   */
  public void setSetpoint(double point) {
    SETPOINTANGLE = MathUtil.clamp(point, CoralArmConstants.MINROTATE, CoralArmConstants.MAXROTATE);
    coralArmPIDController.setSetpoint(point);
  }

  /**
   * Sets the current enum of where the arm should be and how it should intake / outake coral
   * 
   *  @param pos: the num value passed in
   */
  public void setPosition(IntakePosition pos) {
    CoralIntakeSubsystem.intakeMode = pos;
  }

  /**
   * Checks if there is coral inside of the intake. 
   * 
   *  @return returns true if there is a coral in the intake
   */
  public BooleanSupplier hasCoral() {
    return () -> m_rangeSensor.getDistance().getValueAsDouble() < .45;
  }

   // public boolean aroundAngle(double degrees){
  //   return aroundAngle(degrees, 0.2);
  // }

  // public boolean aroundAngle(double degrees, double tolerance){
  //     return MathUtil.isNear(degrees, doubleMeasurement(), tolerance);
  // }

  @ Override
  public void periodic() {}
}
