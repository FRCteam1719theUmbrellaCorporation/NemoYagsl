// /*
//  * Changes the intake angle to the a provided setpoint
//  * 
//  * Runs constantly. 
//  * 
//  * To change the angle, please change the subsystems enum value
//  */

// package frc.robot.commands.Intake;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.AbsoluteEncoder;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.AlgaeArmConstants;
// import edu.wpi.first.math.MathUtil;

// import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;

// public class AlgaePivotPIDCommand extends Command {
//     private final AlgaeIntakeSubsystem intake;
//     private final PIDController m_ArmAngleManager;
//     public double SETPOINTANGLE;
//     private final AbsoluteEncoder m_ANGLE_ENCODER;

//     public AlgaePivotPIDCommand(AlgaeIntakeSubsystem intake) {
//         addRequirements(intake);
//         this.intake = intake;
//         this.m_ArmAngleManager = intake.getPidController();
//         this.m_ANGLE_ENCODER = intake.getEncoder();
//         SETPOINTANGLE = intake.getSetPoint();
//         m_ArmAngleManager.setTolerance(5,10);
//         // System.out.println(SETPOINTANGLE);
//     }
    
//     @Override
//     public void initialize(){
//         // m_ArmAngleManager.reset();
//         m_ArmAngleManager.setSetpoint(SETPOINTANGLE);
//     }

//     @Override
//     public void execute() {
//         double output = MathUtil.clamp(-m_ArmAngleManager.calculate(intake.doubleMeasurement()), AlgaeArmConstants.MIN_SPEED, AlgaeArmConstants.MAX_SPEED);
//         intake.setRotation(output);
//     }
    
//     // public void setAngle(double setpoint) {
//     //     intake.setArmAngle(setpoint);
//     //   } 
//     //TODO Set limits to angle it can reach

//     @Override
//     public void end(boolean interupt) {
//         // this should always run sorry gang
//     }

//     @Override
//     public boolean isFinished() {
//       return false;
//     }
// }
