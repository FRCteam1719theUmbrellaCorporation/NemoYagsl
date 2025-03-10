// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// unused :)

// /**
//  * TODO: PID CONTROLLERS. THEY ARE NOT IMPLEMENTED RIGHT NOW AND ITS IMPORTANT
//  * 
//  * 
//  */
// package frc.robot.subsystems.intake;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.AlgaeArmConstants;
// import frc.robot.Constants.ElevatorConstants;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// // import com.revrobotics.CANSparkMax;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// public class AlgaeIntakeSubsystem extends SubsystemBase {

//   public static enum IntakePosition {
//     ALGAE, MAX, FREE_CONTROL
//   }

//   /** Creates a new IntakeSubsystem. */
//   private static SparkMax TURNMOTOR;
//   private static SparkMax ANGLEMOTOR;
  
//   // temp for now 
//   private static AbsoluteEncoder ANGLE_ENCODER;
//   private static double SETPOINTANGLE;
//   // private boolean intakingPipes;
  
//   private static IntakePosition intakeMode;
  
//   private PIDController ArmAngleManager= new PIDController(AlgaeArmConstants.AlgaeArm_kP, AlgaeArmConstants.AlgaeArm_kI, AlgaeArmConstants.AlgaeArm_kD);  
//   // private final ElevatorFeedforward AlgaeAnglePIDfeed = new ElevatorFeedforward(AlgaeArmConstants.AlgaeArm_kS, AlgaeArmConstants.AlgaeArm_kG, AlgaeArmConstants.AlgaeArm_kV, AlgaeArmConstants.AlgaeArm_kA);


//     public AlgaeIntakeSubsystem() {
//       TURNMOTOR = new SparkMax(Constants.ALGAE_ARM_WHEEL_SPIN_ID, MotorType.kBrushless);
//       ANGLEMOTOR = new SparkMax(Constants.ALGAE_ARM_ANGLE_MOTOR_ID, MotorType.kBrushless);
//       ArmAngleManager.enableContinuousInput(0,1);
//       ANGLE_ENCODER = ANGLEMOTOR.getAbsoluteEncoder();
//       intakeMode = IntakePosition.ALGAE; 
//       SETPOINTANGLE = Constants.CoralArmConstants.ARM_OUTAKE_POS_ANGLE;
//     }
      
//     public void turnIntakeWheels(double speed) {
//       TURNMOTOR.set(speed);
//     }

//     public void stopIntakeMotors() {
//       TURNMOTOR.set(0);
//     }

//     public void setRotation(double angle) {
//       ANGLEMOTOR.set(angle);
//     }

//     public void stopRotation(double angle) {
//       ANGLEMOTOR.set(0);
//     }

//     public void setSetpoint(double point) {
//       SETPOINTANGLE = point;
//       ArmAngleManager.setSetpoint(point);
//     }

//     // returns the angle at which the things should move
//     public double doubleMeasurement() {
//       // System.out.println(ANGLE_ENCODER.getPosition());
//       return ANGLE_ENCODER.getPosition();
//     }

//     // public void reachGoal(double goalDegrees) {
//     //   ANGLEMOTOR.set(ArmAngleManager.calculate(ArmAngleManager.getSetpoint().position, ArmAngleManager.getSetpoint().velocity) + ArmAngleManager.calculate(ANGLE_ENCODER.getPosition(), goalDegrees));
//     // }

//     // public Command setGoal(double goalDegrees) {
//     //     return run(() -> reachGoal(goalDegrees));
//     // }

//     // public Command setArmAngle(double goalDegrees) {
//     //     return setGoal(goalDegrees).until(() -> aroundAngle(goalDegrees));
//     // }

//    public boolean aroundAngle(double degrees){
//         return aroundAngle(degrees, 0.2);
//    }

//    public boolean aroundAngle(double degrees, double tolerance){
//         return MathUtil.isNear(degrees, doubleMeasurement(),tolerance);
//    }

//     // returns if this is intaking
//     public boolean isIntaking() {
//       return intakeMode == IntakePosition.ALGAE;
//     }

//     // returns the current mode
//     public IntakePosition currentMode() {
//       return intakeMode;
//     }

//     public AbsoluteEncoder getEncoder() {
//       return ANGLE_ENCODER;
//     }

//     public double getSetPoint() {
//       return SETPOINTANGLE;
//     }

//     public PIDController getPidController() {
//       return ArmAngleManager;
//     }

//     // public void toggleIntake() {
//     //   intakingPipes = !intakingPipes;
//     // }

//   @Override
//   public void periodic() {

//   }
// }
