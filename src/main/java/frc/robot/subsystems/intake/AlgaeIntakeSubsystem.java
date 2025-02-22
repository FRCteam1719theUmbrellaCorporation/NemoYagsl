// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * TODO: PID CONTROLLERS. THEY ARE NOT IMPLEMENTED RIGHT NOW AND ITS IMPORTANT
 * 
 * 
 */
package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeIntakeSubsystem extends SubsystemBase {

  public static enum IntakePosition {
    ALGAE, MAX, FREE_CONTROL
  }

  /** Creates a new IntakeSubsystem. */
  private static SparkMax TURNMOTOR;
  private static SparkMax ANGLEMOTOR;
  
  // temp for now 
  private static AbsoluteEncoder ANGLE_ENCODER;
  private static double SETPOINTANGLE;
  // private boolean intakingPipes;
  
  private static IntakePosition intakeMode;
  
  private ProfiledPIDController ArmAngleManager= new ProfiledPIDController(AlgaeArmConstants.AlgaeArm_kP, AlgaeArmConstants.AlgaeArm_kI, AlgaeArmConstants.AlgaeArm_kD, new Constraints(ElevatorConstants.MaxVelocity, ElevatorConstants.MaxAcceleration));
  private final ElevatorFeedforward AlgaeAnglePIDfeed = new ElevatorFeedforward(AlgaeArmConstants.AlgaeArm_kS, AlgaeArmConstants.AlgaeArm_kG, AlgaeArmConstants.AlgaeArm_kV, AlgaeArmConstants.AlgaeArm_kA);


    public AlgaeIntakeSubsystem() {
      TURNMOTOR = new SparkMax(Constants.ALGAE_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
      ANGLEMOTOR = new SparkMax(Constants.ALGAE_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);
      ANGLE_ENCODER = TURNMOTOR.getAbsoluteEncoder();
      intakeMode = IntakePosition.ALGAE; 
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
      return ANGLE_ENCODER.getPosition() * 360;
    }

    public void reachGoal(double goalDegrees) {
      ANGLEMOTOR.setVoltage(AlgaeAnglePIDfeed.calculate(ArmAngleManager.getSetpoint().position, ArmAngleManager.getSetpoint().velocity) + ArmAngleManager.calculate(ANGLE_ENCODER.getPosition(), goalDegrees));
    }

    public Command setGoal(double goalDegrees) {
        return run(() -> reachGoal(goalDegrees));
    }

    public Command setArmAngle(double goalDegrees) {
        return setGoal(goalDegrees).until(() -> aroundAngle(goalDegrees));
    }

   public boolean aroundAngle(double degrees){
        return aroundAngle(degrees, 0.2);
   }

   public boolean aroundAngle(double degrees, double tolerance){
        return MathUtil.isNear(degrees, doubleMeasurement(),tolerance);
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
    ANGLEMOTOR.set(ArmAngleManager.calculate(ANGLEMOTOR.getAbsoluteEncoder().getPosition(), SETPOINTANGLE));
  }
}
