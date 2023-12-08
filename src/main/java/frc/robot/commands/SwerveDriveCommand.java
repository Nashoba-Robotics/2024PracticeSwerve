package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.util.JoystickValues;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    public SwerveDriveCommand() {
        addRequirements(
            SwerveDriveSubsystem.getInstance(),
            JoystickSubsystem.getInstance()
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
        JoystickValues moveInput = JoystickSubsystem.getInstance().getLeftJoystickValues().shape(
            Constants.Joystick.MOVE_DEAD_ZONE,
            Constants.Joystick.MOVE_SENSITIVITY
            )
            .swap()
            .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);

        //Takes in and processes the Joystick input for the rotational movement (spinning)
        double rotationInput = JoystickSubsystem.getInstance().getRightJoystickValues().shape(
            Constants.Joystick.TURN_DEAD_ZONE,
            Constants.Joystick.TURN_SENSITIVITY
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