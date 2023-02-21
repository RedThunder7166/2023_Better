package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.util.ClawUtils;
import frc.robot.Constants.Clawstants;

public class Wrist {
    private WPI_TalonFX m_wristMotor;
    private double wristAngleInEncoderUnits = 0;
    private double wristAngleInDegrees = 0;

    private static Wrist singleton;

    private Wrist() {
        m_wristMotor = new WPI_TalonFX(Constants.Clawstants.ClawMotorWristID);

        m_wristMotor.config_kP(0, 0.2);
        m_wristMotor.config_kI(0, 0);
        m_wristMotor.config_kD(0, .1);
        m_wristMotor.configClosedLoopPeakOutput(0, .2);

        m_wristMotor.setInverted(true);

        m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        m_wristMotor.setNeutralMode(NeutralMode.Coast);

        m_wristMotor.configMotionAcceleration(6000);
        m_wristMotor.configMotionCruiseVelocity(7000);
    }

    //Since we only have 1 arm, there is never a reason to have multiple arm objects. Thus, we use a singleton.
    public static Wrist getInstance(){
        if (singleton == null){
            singleton = new Wrist();
        }
        
        return singleton;
        
    }

    public void setAngle(double wristAngle){
        wristAngleInDegrees = wristAngle;
        this.wristAngleInEncoderUnits = ClawUtils.degreesToEncoderUnits(wristAngle, Constants.Clawstants.wristGearRatio);

        m_wristMotor.set(
         ControlMode.MotionMagic,
         this.wristAngleInEncoderUnits,
         DemandType.ArbitraryFeedForward,
         Clawstants.wristFeedForward * java.lang.Math.cos(ClawUtils.encoderUnitsToDegrees(Math.toRadians(m_wristMotor.getSelectedSensorPosition()), Constants.Clawstants.wristGearRatio)));
    }

    public void zeroEncoder() {
        m_wristMotor.setSelectedSensorPosition(0);
        // m_armMotorRight.setSelectedSensorPosition(0);
      }

    public double getAngle(){
        return wristAngleInDegrees;
    }

    public double getEncoderUnits(){
        return wristAngleInEncoderUnits;
    }

    public double getRawEncoderUnits(){
        return m_wristMotor.getSelectedSensorPosition();
    }
}
