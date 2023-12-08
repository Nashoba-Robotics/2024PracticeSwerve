package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.Pigeon2Configurator;
import com.ctre.phoenixpro.hardware.Pigeon2;

import frc.robot.Constants;

public class GyroIOPigeon2 implements GyroIO {
    
    private Pigeon2 gyro;
    private Pigeon2Configurator gyroConfigurator;
    private Pigeon2Configuration gyroConfig;

    public GyroIOPigeon2() {
        gyro = new Pigeon2(Constants.Misc.GYRO_PORT);
        gyroConfigurator = gyro.getConfigurator();
        config();

    }

    private void config() {
        gyroConfig = new Pigeon2Configuration();
        gyroConfig.MountPose.MountPoseYaw = -89.519348;
        gyroConfig.MountPose.MountPosePitch = -0.878906;
        gyroConfig.MountPose.MountPoseRoll = 0;
        gyroConfigurator.apply(gyroConfig);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getYaw().getValue();
        inputs.pitch = gyro.getPitch().getValue();
        inputs.roll = gyro.getRoll().getValue();
    }

    public void setYaw(double angle) {
        gyro.setYaw(angle);
    }

}