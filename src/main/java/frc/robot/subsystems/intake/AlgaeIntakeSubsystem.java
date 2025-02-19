// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * TODO: PID CONTROLLERS. THEY ARE NOT IMPLEMENTED RIGHT NOW AND ITS IMPORTANT
 * 
 * 
 */
package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

public class AlgaeIntakeSubsystem extends SubsystemBase {

  public static enum IntakePosition {
    ALGAE, MAX, FREE_CONTROL
  }

  /** Creates a new IntakeSubsystem. */
  private static SparkMax TURNMOTOR;
  private static SparkMax ANGLEMOTOR;
  
  // temp for now 
  private static DutyCycleEncoder ANGLE_ENCODER;
  private static PIDController ArmAngleManager;
  private static double SETPOINTANGLE;
  // private boolean intakingPipes;
  
  private static IntakePosition intakeMode;

    public AlgaeIntakeSubsystem() {
      this.TURNMOTOR = new SparkMax(Constants.ALGAE_ARM_WHEEL_SPIN_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
      this.ANGLEMOTOR = new SparkMax(Constants.ALGAE_ARM_ANGLE_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
      this.ANGLE_ENCODER = new DutyCycleEncoder(Constants.INTAKE_ENCODER_ANGLE_MOTOR);

      this.ArmAngleManager = new PIDController(Constants.ARMANGLE_kP, Constants.ARMANGLE_kI, Constants.ARMANGLE_kD);
      this.intakeMode = IntakePosition.ALGAE; 
    }
    
    public void turnIntakeWheels(double speed) {
      TURNMOTOR.set(speed);
    }

    public void stopIntakeMotors() {
      TURNMOTOR.set(0);
    }

    public void setRotation(double angle) {
      ANGLEMOTOR.set(angle);
    }

    public void stopRotation(double angle) {
      ANGLEMOTOR.set(0);
    }

    // returns the angle at which the things should move
    public double doubleMeasurement() {
      return this.ANGLE_ENCODER.get() * 360;
    }

    // returns if this is intaking
    public boolean isIntaking() {
      return intakeMode == IntakePosition.ALGAE;
    }

    // returns the current mode
    public IntakePosition currentMode() {
      return intakeMode;
    }

    // public void toggleIntake() {
    //   intakingPipes = !intakingPipes;
    // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ANGLEMOTOR.set(ArmAngleManager.calculate(ANGLE_ENCODER.get(), SETPOINTANGLE));
  }
}
