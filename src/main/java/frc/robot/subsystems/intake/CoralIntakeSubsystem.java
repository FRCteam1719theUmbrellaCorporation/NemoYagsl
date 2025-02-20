// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import frc.robot.Constants;
import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private PIDController coralArmPIDController = new PIDController(CoralArmConstants.CoralArm_kP, 
                                                                    CoralArmConstants.CoralArm_kI, 
                                                                    CoralArmConstants.CoralArm_kD); 
                                                                    // TODO ADD CONSTANT 

  private static SparkMax CORAL_ARM_TURNMOTOR;
  private static SparkMax CORAL_ARM_ANGLEMOTOR;
  private static AbsoluteEncoder CORAL_ARM_ENCODER;

  public CoralIntakeSubsystem() {
    CORAL_ARM_TURNMOTOR = new SparkMax(Constants.CORAL_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
    CORAL_ARM_ANGLEMOTOR = new SparkMax(Constants.CORAL_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);
    CORAL_ARM_ENCODER = CORAL_ARM_ANGLEMOTOR.getAbsoluteEncoder();

  }

  public void intakeSpinWheels(double speed){
    CORAL_ARM_TURNMOTOR.set(speed);
  }

  public void turnOffIntakeWheel(){
    CORAL_ARM_TURNMOTOR.set(0);
  }

  public double doubleMeasurement() {
    return CORAL_ARM_ENCODER.getPosition() * 360;
  }

 /*  public void intakeAngle(double angle){
    CORAL_ARM_ANGLEMOTOR.getAbsoluteEncoder().getPosition();
  }


/* 
  public void resetAngle(){
    CORAL_ARM_ANGLEMOTOR.set(0);
  }
*/



  @ Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
