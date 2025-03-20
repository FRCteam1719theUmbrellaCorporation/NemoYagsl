// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndefectorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import utils.DirectionalPIDController;

/**
 * Controls the robots outake, aka the endeffector
 * 
 * This endeffector only manages 1 motor, the wrist motor. The actual outake method is passive!
 * This subsystem also controls the constant movement of the real endeffector via it's periodic
 */
public class EndEffectorSubsytem extends SubsystemBase {
  // HARDWARE
  private final SparkMax ENDEFFECTOR_ROTATE_MOTOR;
  private final AbsoluteEncoder ENDEFFECTOR_ENCODER; // abs encoder built into spark

  // these are where the robot wants to be positioned
  private double setPoint;
  private HeightLevels heightLevels;
  private Boolean moveDirection; // + is true, - is false, null is fastest
  private DirectionalPIDController EndEffectorPIDController = new DirectionalPIDController(EndefectorConstants.Endefector_kP, 
                                                                    EndefectorConstants.Endefector_kI, 
                                                                    EndefectorConstants.Endefector_kD); 
                                                                    // TODO ADD CONSTANT VALUES

 
  /** Creates a new EndEffectorSubsytem. */
  public EndEffectorSubsytem() {
    // defines hardware
    ENDEFFECTOR_ROTATE_MOTOR = new SparkMax(Constants.ENDEFECTOR_ANGLE_MOTOR_ID, MotorType.kBrushless); 
    ENDEFFECTOR_ENCODER = ENDEFFECTOR_ROTATE_MOTOR.getAbsoluteEncoder();

    // sets the PID settings, and inital setpoint, so the arm always points up on init
    setPoint = 0;
    EndEffectorPIDController.setSetpoint(setPoint);
    EndEffectorPIDController.enableContinuousInput(0,1);
    EndEffectorPIDController.setTolerance(.1);
    heightLevels = HeightLevels.ZERO;
    moveDirection = null;
  }

  /**
   * Stops the endeffector
   */
  public void stop() {
    ENDEFFECTOR_ROTATE_MOTOR.set(0);
  }
  /**
   * Returns the encoder measurement position. ranges from 0 to 1
   */
  public double doubleMeasurement() {
    return ENDEFFECTOR_ENCODER.getPosition();
  }

  /**
   * Sets the rotation of the endeffector motor
   */
  public void setRotation(double angle) {
    ENDEFFECTOR_ROTATE_MOTOR.set(angle);
  }
  
  /**
  * Returns the PID controller from the endeffector
  * 
  * @return Returns the robots PID controller
  */
  public DirectionalPIDController getPID() {
    return EndEffectorPIDController;
  }

  /**
   * Returns the Endeffector's desired setpoint
   * 
   * @return Returns the desired setpoint
   */
  public double getSetpoint() {
    return setPoint;
  }

  /**
   * Sets the height level
   * 
   * @param desired height level
   */
  public void setHeight(HeightLevels level) {
    heightLevels = level;
  }

  public void setDirection(Boolean dir) {
    moveDirection = dir;
  }

  /**
   * Sets the desired setpoint for the endeffector. This also accounts for values outside the range of 0 to 1
   * 
   * @param point: desired setpoint. will be clamped to 0 to 1
   */
  public void setSetpoint(double point) {
    setPoint = MathUtil.clamp(point, 0, 1);
    EndEffectorPIDController.setSetpoint(setPoint);
  }

  /**
   * Returns the set height level
   * 
   * @return heightlevel 
   */
  public HeightLevels getHeight() {
    return heightLevels;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    double output = MathUtil.clamp(EndEffectorPIDController.calculate(doubleMeasurement(), moveDirection),EndefectorConstants.MIN_SPEED, EndefectorConstants.MAX_SPEED);
    // System.out.println("endeff output: " + output);
    setRotation(output);
  }
}
 