// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import frc.robot.Constants;
import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

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

  private static SparkMax CORAL_ARM_TURNMOTOR;
  private static SparkMax CORAL_ARM_ANGLEMOTOR;
  private static AbsoluteEncoder CORAL_ARM_ENCODER;
  private static double SETPOINTANGLE;
  private static IntakePosition intakeMode;


  public CoralIntakeSubsystem() {
    CORAL_ARM_TURNMOTOR = new SparkMax(Constants.CORAL_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
    CORAL_ARM_ANGLEMOTOR = new SparkMax(Constants.CORAL_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);
    CORAL_ARM_ENCODER = CORAL_ARM_ANGLEMOTOR.getAbsoluteEncoder();
    coralArmPIDController.enableContinuousInput(0,1);
    SETPOINTANGLE = CoralArmConstants.coral_armdriving_pos;
    intakeMode = IntakePosition.DRIVING; 

    coralArmPIDController.setTolerance(5,10);
  }

  public void intakeSpinWheels(double speed){
    CORAL_ARM_TURNMOTOR.set(speed);
  }

  public void turnOffIntakeWheel(){
    CORAL_ARM_TURNMOTOR.set(0);
  }
  
  public double doubleMeasurement() {
    return CORAL_ARM_ENCODER.getPosition();
  }

  public void setRotation(double angle) {
    CORAL_ARM_ANGLEMOTOR.set(angle);
  }

  public void stopRotation(double angle) {
    CORAL_ARM_ANGLEMOTOR.set(0);
  }

  public boolean isIntaking() {
    return intakeMode == IntakePosition.FLOOR;
  }

  // returns the current mode
  public IntakePosition currentMode() {
    return intakeMode;
  }

  public AbsoluteEncoder getEncoder() {
    return CORAL_ARM_ENCODER;
  }

  public double getSetpoint() {
    return SETPOINTANGLE;
  }

  public PIDController getPID() {
    return coralArmPIDController;
  }  

  public void setSetpoint(double point) {
    SETPOINTANGLE = MathUtil.clamp(point, CoralArmConstants.MINROTATE, CoralArmConstants.MAXROTATE);
    coralArmPIDController.setSetpoint(point);
  }

  public void setPosition(IntakePosition pos) {
    this.intakeMode = pos;
  }

  public boolean aroundAngle(double degrees){
    return aroundAngle(degrees, 0.2);
  }

  public boolean aroundAngle(double degrees, double tolerance){
      return MathUtil.isNear(degrees, doubleMeasurement(), tolerance);
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
    // CORAL_ARM_ANGLEMOTOR.set(coralArmPIDController.calculate(CORAL_ARM_ANGLEMOTOR.getAbsoluteEncoder().getPosition(), SETPOINTANGLE));

  }
}
