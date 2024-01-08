package frc.robot.subsystems.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.lib.util.JoystickValues;

public class ControllerSubsystem implements OperatorInput{
    private static ControllerSubsystem instance;
    public static ControllerSubsystem getInstance(){
        if(instance == null) instance = new ControllerSubsystem();
        return instance;
    }

    CommandJoystick controller = new CommandJoystick(2);
    
    public JoystickValues getLeftJoystickValues(){
        return new JoystickValues(controller.getX(), -controller.getY());
    }
    public JoystickValues getRightJoystickValues(){
        return new JoystickValues(controller.getZ(), controller.getTwist());
    }
}
