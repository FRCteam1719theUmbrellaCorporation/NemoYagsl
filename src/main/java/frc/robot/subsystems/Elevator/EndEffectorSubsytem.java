// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndefectorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;


public class EndEffectorSubsytem extends SubsystemBase {

  private final SparkMax ENDEFFECTOR_ROTATE_MOTOR;
  private final AbsoluteEncoder ENDEFFECTOR_ENCODER;
  private double setPoint;
  private PIDController EndEffectorPIDController = new PIDController(EndefectorConstants.Endefector_kP, 
                                                                    EndefectorConstants.Endefector_kI, 
                                                                    EndefectorConstants.Endefector_kD); 
                                                                    // TODO ADD CONSTANT VALUES

 
  /** Creates a new EndEffectorSubsytem. */
  public EndEffectorSubsytem() {
    ENDEFFECTOR_ROTATE_MOTOR = new SparkMax(Constants.ENDEFECTOR_ANGLE_MOTOR_ID, MotorType.kBrushless); 
    ENDEFFECTOR_ENCODER = ENDEFFECTOR_ROTATE_MOTOR.getAbsoluteEncoder();
    setPoint = 0;
    EndEffectorPIDController.setSetpoint(setPoint);
  }

  public void stop() {
    ENDEFFECTOR_ROTATE_MOTOR.set(0);
  }

  public double doubleMeasurement() {
    return ENDEFFECTOR_ENCODER.getPosition();
  }

  public void setRotation(double angle) {
    ENDEFFECTOR_ROTATE_MOTOR.set(angle);
  }

  public void stopRotation(double angle) {
    ENDEFFECTOR_ROTATE_MOTOR.set(0);
  }


  @Override
  public void periodic() {
    double output = MathUtil.clamp(EndEffectorPIDController.calculate(doubleMeasurement()),EndefectorConstants.MIN_SPEED, EndefectorConstants.MAX_SPEED);
    System.out.println("ENDEFFECTOR DISABLED. TUNE PIDS PLEASE!");
    // setRotation(output);
    // This method will be called once per scheduler run
  }
}
