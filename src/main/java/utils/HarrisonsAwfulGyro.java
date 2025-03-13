package utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.spns.SpnValue;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public class HarrisonsAwfulGyro extends Pigeon2 {

    private final StatusSignal<Angle> m_yawGetter = getYaw(false).clone();
    private final StatusSignal<AngularVelocity> m_yawRateGetter = getAngularVelocityZWorld(false).clone();
    private final StatusSignal<Double> m_quatWGetter = getQuatW(false).clone();
    private final StatusSignal<Double> m_quatXGetter = getQuatX(false).clone();
    private final StatusSignal<Double> m_quatYGetter = getQuatY(false).clone();
    private final StatusSignal<Double> m_quatZGetter = getQuatZ(false).clone();

    HarrisonsAwfulGyro(int id) {
        this(id, "");
    }

    HarrisonsAwfulGyro(int id, CANBus bus) {
        this(id, bus.getName());
    }

    HarrisonsAwfulGyro(int id, String canbus){
        super(id, canbus);
    }

    @Override
    public double getAngle() {
        return -(m_yawGetter.refresh().getValueAsDouble() - m_yawGetter.refresh().getValueAsDouble()*0.03);
    }    

    // @Override
    // public StatusSignal<Angle> getYaw(boolean refresh)
    // {
    //     super.getYaw(refresh);

    // }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(m_yawGetter.refresh().getValueAsDouble() - m_yawGetter.getValueAsDouble() *Constants.GYRO_OFFSET_MULT);
    }
    
    @Override 
    public Rotation3d getRotation3d() {
        BaseStatusSignal.refreshAll(m_quatWGetter, m_quatXGetter, m_quatYGetter, m_quatZGetter);
        return new Rotation3d(new Quaternion(m_quatWGetter.getValue(), m_quatXGetter.getValue(), m_quatYGetter.getValue(), m_quatZGetter.getValue() - m_quatZGetter.getValue() * Constants.GYRO_OFFSET_MULT));
    }

}
