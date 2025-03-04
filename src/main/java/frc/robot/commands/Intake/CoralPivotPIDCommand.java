
//  * Changes the intake angle to the a provided setpoint
//  * 
//  * Runs constantly. 
//  * 
//  * To change the angle, please change the subsystems enum value
//  */

 package frc.robot.commands.Intake;

 import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.Constants.CoralArmConstants;
import edu.wpi.first.math.MathUtil;

public class CoralPivotPIDCommand extends Command {
    private final CoralIntakeSubsystem intake;
    private final PIDController m_ArmAngleManager;
    private final double SETPOINTANGLE;
    private final AbsoluteEncoder m_ANGLE_ENCODER;
    private CoralIntakeSubsystem.IntakePosition previousPos;
    private final CoralIntakeWheelsCommand wheelsWoooo;
    public CoralPivotPIDCommand(CoralIntakeSubsystem intake) {
        //throw new Exception();

        addRequirements(intake);
        this.intake = intake;
        this.m_ArmAngleManager = intake.getPID();
        this.SETPOINTANGLE = intake.getSetpoint();
        this.m_ANGLE_ENCODER = intake.getEncoder();
        this.previousPos = null;
        this.wheelsWoooo = new CoralIntakeWheelsCommand(intake);

    }
    
    @Override
    public void initialize(){
        m_ArmAngleManager.setSetpoint(SETPOINTANGLE);
    }

    @Override
    public void execute() {
        // TODO: ADD AND FIND SETPOINTS HERE

        if (intake.currentMode() != previousPos) {
            previousPos = intake.currentMode();

            //wheelsWoooo.turnMotor();
            switch (intake.currentMode()) {

                case DRIVING:
                    intake.setSetpoint(CoralArmConstants.coral_armdriving_pos);
                    break;
                
                case FLOOR:
                    intake.setSetpoint(CoralArmConstants.coral_floorintake_pos);
                    break;
                
                case HUMAN_STATION:
                    intake.setSetpoint(CoralArmConstants.coral_humanstatione_pos);
                    break;
                
                case REEF:
                    intake.setSetpoint(CoralArmConstants.coral_reef_l1);
                    break;
            
                default:
                    intake.setSetpoint(CoralArmConstants.coral_armdriving_pos);
                    break;
            }
        }
        
        double output = MathUtil.clamp(-m_ArmAngleManager.calculate(m_ANGLE_ENCODER.getPosition()), CoralArmConstants.MIN_SPEED, CoralArmConstants.MAX_SPEED);
       // System.out.println(output);
        //System.out.println(m_ANGLE_ENCODER.getPosition());
        intake.setRotation(output);

        // // m_ArmAngleManager.setSetpoint(intake.isIntaking() ? Constants.IntakeDetails.intakePos : SETPOINTANGLE.getAsDouble());
        // System.out.println(m_ArmAngleManager.calculate(m_ArmAngleManager.calculate(m_ANGLE_ENCODER.get())));
        // intake.setRotation(m_ArmAngleManager.calculate(m_ArmAngleManager.calculate(m_ANGLE_ENCODER.get())));
    }

    @Override
    public void end(boolean interupt) {
        // this should always run sorry gang
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
