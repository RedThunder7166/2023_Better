package frc.lib.util;

public class ClawUtils {
    public static double degreesToEncoderUnits(double angle, double gearRatio) {
        double encoderUnitsPerMotorRotation = 2048;
        double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation;
    
        return angle * (encoderUnitsPerArmRotation / 360); //desired angle in degrees * encoder units per degree
    
      }
    
      public static double encoderUnitsToDegrees(double encoderUnits, double gearRatio) {
        // double gearRatio = 144;
        double encoderUnitsPerMotorRotation = 2048;
        double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation;
    
        return (encoderUnits * 360) / encoderUnitsPerArmRotation;
      }
}
