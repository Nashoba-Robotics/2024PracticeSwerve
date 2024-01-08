package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;
import frc.robot.subsystems.joystick.ControllerSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.joystick.OperatorInput;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {

    // private OperatorInput joystick = JoystickSubsystem.getInstance();
    private OperatorInput joystick = ControllerSubsystem.getInstance();

    double MOVE_SENSITIVITY = joystick instanceof JoystickSubsystem ? 
                                Constants.Joystick.MOVE_SENSITIVITY : 
                                2.5;
    double MOVE_DEAD_ZONE = joystick instanceof JoystickSubsystem ? 
                                Constants.Joystick.MOVE_DEAD_ZONE : 
                                0.01;
    
    double TURN_SENSITIVITY = joystick instanceof JoystickSubsystem ? 
                                Constants.Joystick.TURN_SENSITIVITY : 
                                1.0;
    double TURN_DEAD_ZONE = joystick instanceof JoystickSubsystem ? 
                                Constants.Joystick.TURN_DEAD_ZONE : 
                                0.01;


    public SwerveDriveCommand() {
        addRequirements(
            SwerveDriveSubsystem.getInstance()
        );

    }

    @Override
    public void initialize() {
        //Resets all the wheels to face forwards
        SwerveDriveSubsystem.getInstance().resetModulesAbsolute();
    }

    @Override
    public void execute() {
        //Takes in and processes the Joystick input for drive movement
        JoystickValues moveInput = joystick.getLeftJoystickValues().shape(
            MOVE_DEAD_ZONE,
            MOVE_SENSITIVITY
            )
            .swap()
            .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);

        //Takes in and processes the Joystick input for the rotational movement (spinning)
        double rotationInput = joystick.getRightJoystickValues().shape(
            TURN_DEAD_ZONE,
            TURN_SENSITIVITY
        ).x;

        //Tells the Swerve Drive what to do
        SwerveDriveSubsystem.getInstance().set(
            moveInput,
            rotationInput
        );

    }

    @Override
    public void end(boolean interrupted) {

    }
 
    @Override
    public boolean isFinished() {
        return false;
    }

}