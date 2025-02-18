// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class EndEffectorSubsytem extends SubsystemBase {

  private final SparkMax OUTAKE_ROTATE_MOTOR;
  private final DutyCycleEncoder OUTAKE_ROTATE_ENCODER;
  private PIDController rotationPIDController = new PIDController(0, 0, 0); // TODO ADD CONSTANT VALUES
 
  /** Creates a new EndEffectorSubsytem. */
  public EndEffectorSubsytem() {
    OUTAKE_ROTATE_MOTOR = new SparkMax(Constants.ELEVATOR_PIN_TWO, MotorType.kBrushless); 
    OUTAKE_ROTATE_ENCODER = new DutyCycleEncoder(Constants.ELEVATOR_ROTATE_ENCODER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
