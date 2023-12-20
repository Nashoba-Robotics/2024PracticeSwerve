package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.math.SwerveMath;
import frc.robot.lib.util.CarpetOdometry;
import frc.robot.lib.util.JoystickValues;
import frc.robot.lib.util.SwerveState;

public class SwerveDriveSubsystem extends SubsystemBase{
    private CarpetOdometry odometry;
    private Module[] modules;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private boolean fieldCentric;

    private PIDController balanceController;
    private PIDController cardinalController;

    private SwerveDriveSubsystem() {
        gyroIO = new GyroIOPigeon2();

        fieldCentric = true;

        modules = new Module[] {
            new Module(0, Constants.Swerve.FRONT_RIGHT_MOVE_PORT, Constants.Swerve.FRONT_RIGHT_TURN_PORT, Constants.Swerve.FRONT_RIGHT_SENSOR_PORT, Constants.Swerve.FRONT_RIGHT_OFFSET_NU, Constants.Swerve.MOD0_AFF),
            new Module(1, Constants.Swerve.FRONT_LEFT_MOVE_PORT, Constants.Swerve.FRONT_LEFT_TURN_PORT, Constants.Swerve.FRONT_LEFT_SENSOR_PORT, Constants.Swerve.FRONT_LEFT_OFFSET_NU, Constants.Swerve.MOD1_AFF),
            new Module(2, Constants.Swerve.BACK_LEFT_MOVE_PORT, Constants.Swerve.BACK_LEFT_TURN_PORT, Constants.Swerve.BACK_LEFT_SENSOR_PORT, Constants.Swerve.BACK_LEFT_OFFSET_NU, Constants.Swerve.MOD2_AFF),
            new Module(3, Constants.Swerve.BACK_RIGHT_MOVE_PORT, Constants.Swerve.BACK_RIGHT_TURN_PORT, Constants.Swerve.BACK_RIGHT_SENSOR_PORT, Constants.Swerve.BACK_RIGHT_OFFSET_NU, Constants.Swerve.MOD3_AFF)
        };

        odometry = new CarpetOdometry(Constants.Swerve.KINEMATICS, Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), Constants.Field.ANGLE_OF_RESISTANCE);
    
        balanceController = new PIDController(Constants.Swerve.Balance.SLOW_K_P, Constants.Swerve.Balance.SLOW_K_I, Constants.Swerve.Balance.SLOW_K_D);
        cardinalController = new PIDController(0.01, 0, 0.0);
        cardinalController.setTolerance(2);
    }
    
    private static SwerveDriveSubsystem instance;
    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    private double setAngle = 0;

    /*
     * @param {double} angle - Angle to set the motor to in radians
     */
    public void turnModulesToAngle(double angle) {
        setAngle = NRUnits.constrainRad(angle - getGyroAngle());
        for(Module module : modules) {
            module.turn(setAngle);
        }
    }

    /*
     * @return Whether or not every module has reached the target angle
     */
    public boolean atTurnPos() {
        boolean atPos = true;
        for(Module module : modules) {
            boolean atAngle = module.atTargetAngle(setAngle)
            || module.atTargetAngle(setAngle + Constants.TAU/2)
            || module.atTargetAngle(setAngle - Constants.TAU/2);
            atPos = atPos && atAngle;
        }
        return atPos;
    }

    public double getYaw(){
        return gyroInputs.yaw;
    }
    
    public boolean atCardinalAngle(){
        return cardinalController.atSetpoint();
    }

    public void brake() {
        modules[0].set(0, Constants.TAU/8, false);
        modules[1].set(0, -Constants.TAU/8, false);
        modules[2].set(0, Constants.TAU/8, false);
        modules[3].set(0, -Constants.TAU/8, false);
    }

    public double getGyroAngle() {
        return NRUnits.constrainDeg(getYaw()) * Constants.TAU / 360;
    }

    //Don't know if this will work
    public double getBalanceAngle() {
        double pitch = getPitch();
        double roll = getRoll();
        // What to do if pitch and roll are negative and positive? (I don't think it will happen)
        if(Math.signum(pitch) == -1 && Math.signum(roll) == -1)
        return -Math.sqrt(pitch*pitch + roll*roll);
        return Math.sqrt(pitch*pitch + roll*roll);
    }

    //Convert to radians?
    public double getPitch(){
        return gyroInputs.pitch;
    }

    public double getRoll(){
        return gyroInputs.roll;
    }

    public void set(JoystickValues joystickValues, double omega) {
        set(joystickValues.x, joystickValues.y, omega);
    }

    public void set(double x, double y, double omega) {

        if(fieldCentric) {
            double angleDiff = Math.atan2(y, x) - getGyroAngle(); //difference between input angle and gyro angle gives desired field relative angle
            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(angleDiff);
            y = r * Math.sin(angleDiff);
        }
        
        //Repeated equations
        double a = omega * Constants.Swerve.WIDTH/2;
        double b = omega * Constants.Swerve.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d t0 = new Translation2d(x-b, y-a);
        Translation2d t1 = new Translation2d(x+b, y-a);
        Translation2d t2 = new Translation2d(x+b, y+a);
        Translation2d t3 = new Translation2d(x-b, y+a);

        //convert to polar
        SwerveState[] setStates = SwerveState.fromTranslation2d(
            new Translation2d[] {t0, t1, t2, t3}
        );

        setStates = SwerveMath.normalize(setStates);

        set(setStates);
    }

    public void set(SwerveState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public void setDirectly(double speed, double angle) {
        modules[0].set(speed, angle);
        modules[1].set(speed, angle);
        modules[2].set(speed, angle);
        modules[3].set(speed, angle);
    }

    public void setAngle(double angle){ //rad
        modules[0].turn(angle);
        modules[1].turn(angle);
        modules[2].turn(angle);
        modules[3].turn(angle);
    }

    private boolean resetting = false;

    public void resetOdometry(Pose2d pose) {
        resetting = true;
        odometry.resetPosition(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions(), pose);
        resetting = false;
    }

    public void resetOdometryOverrideAngle(Pose2d pose, Rotation2d angle) {
        resetting = true;
        odometry.resetPosition(angle, getSwerveModulePositions(), pose);
        resetting = false;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void setStates(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            // SmartDashboard.putNumber("SetAngle"+i, states[i].angle.getDegrees());
            modules[i].set(states[i]);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    //radians
    public void setGyro(double angle) {
        gyroIO.setYaw(angle * 180 / Math.PI);
    }

    public void zeroYaw() {
        gyroIO.setYaw(0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double[] getModAngles() {
        return new double[] {
            modules[0].getAbsAngle(),
            modules[1].getAbsAngle(),
            modules[2].getAbsAngle(),
            modules[3].getAbsAngle(),
        };
    }

    public double[] getAnglePositions() {
        return new double[] {
            modules[0].getAngle(),
            modules[1].getAngle(),
            modules[2].getAngle(),
            modules[3].getAngle(),
        };
    }

    //Set the desired angle for the balancing (level) and the allowed error (deadzone)
    //All in degrees
    public void setDesiredLevel(double angle, double deadzone){
        balanceController.setSetpoint(angle);
        balanceController.setTolerance(deadzone);
    }

    public void setBalancePID(double kP, double kI, double kD){
        balanceController.setP(kP);
        balanceController.setI(kI);
        balanceController.setD(kD);
    }

    public boolean balanced(){
        return balanceController.atSetpoint();
    }

    public boolean isLevel(){
        return Math.abs(getRoll()) < 1.5;
    }

    public boolean notLevel() {
        return !isLevel();
    }

    public boolean reallyNotLevel() {
        return !(Math.abs(getRoll()) < 8);
    }

    public boolean levelNegative() {
        return getRoll() < -1;
    }

    public boolean levelPositive() {
        return getRoll() > 1;
    }

    //TODO: Add algorithm to check whether to use Pitch or Roll (Maybe averaging the values?)
    public double getChange(){
        return balanceController.calculate(getRoll());
        //return balanceController.calculate(getBalanceAngle());
    }

    public void resetModulesAbsolute() {
        for(Module module : modules) {
            module.resetTurnToAbsolute();
        }
    }

    public double getXVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return speed.vxMetersPerSecond;
    }

    public double getYVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return speed.vyMetersPerSecond;
    }

    public double getVelocity() {
        ChassisSpeeds speed = Constants.Swerve.KINEMATICS.toChassisSpeeds(
            modules[0].getSwerveState(),
            modules[1].getSwerveState(),
            modules[2].getSwerveState(),
            modules[3].getSwerveState());

        return Math.abs(Math.sqrt(speed.vxMetersPerSecond*speed.vxMetersPerSecond + speed.vyMetersPerSecond*speed.vyMetersPerSecond));
    }

    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        for(Module module : modules) {
            module.updateMovePosition();
            module.periodic();
        }

        if(!resetting) odometry.update(Rotation2d.fromRadians(getGyroAngle()), getSwerveModulePositions());
        Pose2d pose = odometry.getPoseMeters();

        Logger.getInstance().recordOutput("Drive/Odometry/X", pose.getX());
        Logger.getInstance().recordOutput("Drive/Odometry/Y", pose.getY());
        Logger.getInstance().recordOutput("Drive/Odometry/Angle", pose.getRotation().getDegrees());
    
        SmartDashboard.putNumber("Gyro Angle", gyroInputs.yaw);
        
        for(Module module : modules){
            SmartDashboard.putNumber("Module " + module.modIndex + ": Turn Angle", module.getTurnAngle());
        }
        SmartDashboard.putData(instance);
    }

}