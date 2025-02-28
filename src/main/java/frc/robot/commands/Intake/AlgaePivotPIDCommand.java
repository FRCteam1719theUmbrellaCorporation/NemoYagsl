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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;

public class AlgaePivotPIDCommand extends Command {
    private final AlgaeIntakeSubsystem intake;
    private final PIDController m_ArmAngleManager;
    public double SETPOINTANGLE;
    private final AbsoluteEncoder m_ANGLE_ENCODER;

    public AlgaePivotPIDCommand(AlgaeIntakeSubsystem intake) {
        addRequirements(intake);
        this.intake = intake;
        this.m_ArmAngleManager = intake.getPidController();
        this.m_ANGLE_ENCODER = intake.getEncoder();
        SETPOINTANGLE = intake.getSetPoint();
        m_ArmAngleManager.setTolerance(5,10);
        System.out.println(SETPOINTANGLE);
    }
    
    @Override
    public void initialize(){
        // m_ArmAngleManager.reset();
        m_ArmAngleManager.setSetpoint(SETPOINTANGLE);
    }

    @Override
    public void execute() {
        // TODO: ADD AND FIND SETPOINTS HERE
        
        double output = -m_ArmAngleManager.calculate(intake.doubleMeasurement());
        System.out.println(output);
        System.out.println( m_ANGLE_ENCODER.getPosition());
        // intake.setRotation(-.25);
        intake.setRotation(output);

        // switch (intake.currentMode()) {m_

        //     case ALGAE:
        //         break;
            
        //     case MAX:
        //         break;
        
        //     default:
        //         if (this.SETPOINTANGLE > 120) {
        //             SETPOINTANGLE = 120;
        //         }
        // }

        // if (!intake.aroundAngle(SETPOINTANGLE)) {
        //     intake.setArmAngle(SETPOINTANGLE).schedule();
        // }
        // // m_ArmAngleManager.setSetpoint(intake.isIntaking() ? Constants.IntakeDetails.intakePos : SETPOINTANGLE.getAsDouble());
        // if (m_ArmAngleManager.atSetpoint()) {System.out.println(m_ArmAngleManager.atSetpoint());}
        // System.out.println(m_ArmAngleManager.calculate(m_ArmAngleManager.atSetpoint() ? 0 : m_ArmAngleManager.calculate(intake.doubleMeasurement(), SETPOINTANGLE)));

        // intake.setRotation(m_ArmAngleManager.calculate(m_ArmAngleManager.atSetpoint() ? 0 : m_ArmAngleManager.calculate(intake.doubleMeasurement(), SETPOINTANGLE)));
        // // intake.setArmAngle(SETPOINTANGLE);
    }
    
    // public void setAngle(double setpoint) {
    //     intake.setArmAngle(setpoint);
    //   } 
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
