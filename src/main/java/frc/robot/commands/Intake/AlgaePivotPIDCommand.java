/*
 * Changes the intake angle to the a provided setpoint
 * 
 * Runs constantly. 
 * 
 * To change the angle, please change the subsystems enum value
 */

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;

public class AlgaePivotPIDCommand extends Command {
    private final AlgaeIntakeSubsystem intake;
    private final PIDController m_ArmAngleManager;
    private static DoubleSupplier SETPOINTANGLE;
    private final AbsoluteEncoder m_ANGLE_ENCODER;

    public AlgaePivotPIDCommand(AlgaeIntakeSubsystem intake, PIDController m_ArmAngleManager, DoubleSupplier SETPOINTANGLE, AbsoluteEncoder m_ANGLE_ENCODER) {
        addRequirements(intake);
        this.intake = intake;
        this.m_ArmAngleManager = m_ArmAngleManager;
        this.SETPOINTANGLE = SETPOINTANGLE;
        this.m_ANGLE_ENCODER = m_ANGLE_ENCODER;

    }
    
    @Override
    public void initialize(){
        m_ArmAngleManager.setSetpoint(SETPOINTANGLE.getAsDouble());
    }

    @Override
    public void execute() {
        // TODO: ADD AND FIND SETPOINTS HERE
        switch (intake.currentMode()) {

            case ALGAE:
                break;
            
            case MAX:
                break;
        
            default:
                break;
        }

        // m_ArmAngleManager.setSetpoint(intake.isIntaking() ? Constants.IntakeDetails.intakePos : SETPOINTANGLE.getAsDouble());

        intake.setRotation(m_ArmAngleManager.calculate(m_ArmAngleManager.calculate(m_ANGLE_ENCODER.getPosition())));
    }
    
    public void setAngle(double setpoint) {
        m_ArmAngleManager.setSetpoint(setpoint);
      } 
    //TODO Set limits to angle it can reach

    @Override
    public void end(boolean interupt) {
        // this should always run sorry gang
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
