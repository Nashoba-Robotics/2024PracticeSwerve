package frc.robot.subsystems.joystick;

import frc.robot.lib.util.JoystickValues;

public interface OperatorInput{
    public abstract JoystickValues getRightJoystickValues();
    public abstract JoystickValues getLeftJoystickValues();
}
