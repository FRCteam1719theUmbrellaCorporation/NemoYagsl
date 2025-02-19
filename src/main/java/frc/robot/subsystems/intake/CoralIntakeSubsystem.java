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
  private final SparkMax CoralArm_Wheel_Motor;
  private final SparkMax CoralArm_Angle_Motor;

  public CoralIntakeSubsystem() {
    CoralArm_Wheel_Motor = new SparkMax(Constants.CORAL_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
    CoralArm_Angle_Motor = new SparkMax(Constants.CORAL_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);

  }

  public void intakeSpinWheels(double speed){
    CoralArm_Wheel_Motor.set(speed);
  }

  public void turnOffIntakeWheel(){
    CoralArm_Wheel_Motor.set(0);
  }

  /*public void intakeAngle(double angle){
    CoralArm_Angle_Motor.getAbsoluteEncoder().getPosition();
  }

*/
/* 
  public void resetAngle(){
    CoralArm_Angle_Motor.set(0);
  }
*/



  @ Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
