
//  * Changes the intake angle to the a provided setpoint
//  * 
//  * Runs constantly. 
//  * 
//  * To change the angle, please change the subsystems enum value
//  */

 package frc.robot.commands.Intake;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.Constants.CoralArmConstants;

public class CoralPivotPIDCommand extends Command {
    // 
    private final CoralIntakeSubsystem intake;
    private final AbsoluteEncoder m_ANGLE_ENCODER;
    private final PIDController m_ArmAngleManager;
    private final double SETPOINTANGLE;
    private CoralIntakeSubsystem.IntakePosition previousPos;
    // private final CoralIntakeWheelsCommand wheelsWoooo;
    public CoralPivotPIDCommand(CoralIntakeSubsystem intake) {
        addRequirements(intake);
        this.intake = intake;
        this.m_ArmAngleManager = intake.getPID();
        this.SETPOINTANGLE = intake.getSetpoint();
        this.m_ANGLE_ENCODER = intake.getEncoder();
        this.previousPos = null;
    }
    
    @Override
    public void initialize(){
        m_ArmAngleManager.setSetpoint(SETPOINTANGLE);
    }

    @Override
    public void execute() {
        if (intake.currentMode() != previousPos) {
            previousPos = intake.currentMode();
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

        // clamps to max distance and such
        double output = MathUtil.clamp(-m_ArmAngleManager.calculate(m_ANGLE_ENCODER.getPosition()), CoralArmConstants.MIN_SPEED, CoralArmConstants.MAX_SPEED);
        intake.setRotation(output);
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
