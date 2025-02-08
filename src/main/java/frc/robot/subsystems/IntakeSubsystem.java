// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// // import com.revrobotics.CANSparkMax;
// import com.revrobotics.spark.SparkMax;
// // import com.revrobotics.CANSparkLowLevel.MotorType;

// public class IntakeSubsystem extends SubsystemBase {
//   /** Creates a new IntakeSubsystem. */
//   private static SparkMax TURNMOTOR;
//   private static SparkMax ANGLEMOTOR;
    
//   // temp for now 
//   private static DutyCycleEncoder ANGLE_ENCODER;
//   private static PIDController ArmAngleManager;
//   private static double SETPOINTANGLE;

//     public IntakeSubsystem() {
//       this.TURNMOTOR = new SparkMax(Constants.INTAKE_TURN_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
//       this.ANGLEMOTOR = new SparkMax(Constants.INTAKE_ANGLE_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
//       this.ANGLE_ENCODER = new DutyCycleEncoder(Constants.INTAKE_ENCODER_ANGLE_MOTOR);

//       this.ArmAngleManager = new PIDController(Constants.ARMANGLE_kP, Constants.ARMANGLE_kI, Constants.ARMANGLE_kD);
//     }

//     public void turnIntakeWheels(double speed) {
//       TURNMOTOR.set(speed);
//     }

//     public void stopIntakeMotors() {
//       TURNMOTOR.set(0);
//     }

//     public void setArmPos(double angle) {
//       SETPOINTANGLE = angle;
//     }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     ANGLEMOTOR.set(ArmAngleManager.calculate(ANGLE_ENCODER.get(), SETPOINTANGLE));
//   }
// }
