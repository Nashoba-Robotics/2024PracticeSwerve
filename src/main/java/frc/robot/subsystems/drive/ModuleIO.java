package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        public double movePosition = 0; // NU
        public double moveVelocity = 0; // NU/s
        public double moveVoltage = 0; // Volts
        public double moveStatorCurrent = 0; // Amps
        public double moveSupplyCurrent = 0; // Amps

        public double turnAbsolutePosition = 0; // Rot
        public double turnRotorPosition = 0; // NU
        public double turnVelocity = 0; // NU/s
        public double turnVoltage = 0; // Volts
        public double turnStatorCurrent = 0; // Amps
        public double turnSupplyCurrent = 0; // Amps
        public double sensorVelocity = 0; //Rot/s
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setMoveVelocity(double velocity) {}
    public default void setTurnPosition(double position) {}
    public default void setTurnVelocity(double velocity) {}
    
    public default void setkF(double kF) {}
    public default void setkP(double kP) {}
    public default void setkD(double kD) {}


    public default void assignTurnPosition(double position) {}

}