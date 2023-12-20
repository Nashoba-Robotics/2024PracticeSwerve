package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class ModuleIOTalonFX implements ModuleIO {

    private TalonFX moveMotor;
    private TalonFXConfigurator moveConfigurator;
    private TalonFXConfiguration moveConfig;
    private VelocityDutyCycle moveControl;

    private TalonFX turnMotor;
    private TalonFXConfigurator turnConfigurator;
    private TalonFXConfiguration turnConfig;
    private MotionMagicDutyCycle turnControl;
    private VelocityDutyCycle turnVelocityControl;

    private CANcoder sensor;
    private CANcoderConfigurator sensorConfigurator;
    private CANcoderConfiguration sensorConfig;

    private final String CANBUS_NAME = "drivet";

    public ModuleIOTalonFX(int movePort, int turnPort, int sensorPort, double offset, double AFF) {
        moveMotor = new TalonFX(movePort);
        moveConfigurator = moveMotor.getConfigurator();
        moveConfig = new TalonFXConfiguration();

        moveControl = new VelocityDutyCycle(0);
        moveControl.FeedForward = AFF;
        moveControl.EnableFOC = true;
        moveControl.UpdateFreqHz = 50;

        turnMotor = new TalonFX(turnPort);
        turnConfigurator = turnMotor.getConfigurator();
        turnConfig = new TalonFXConfiguration();

        turnControl = new MotionMagicDutyCycle(offset);
        turnControl.UpdateFreqHz = 50;
        
        turnVelocityControl = new VelocityDutyCycle(0);
        turnVelocityControl.Slot = 0;

        sensor = new CANcoder(sensorPort);
        sensorConfigurator = sensor.getConfigurator();
        sensorConfig = new CANcoderConfiguration();


        config();
        // configOffset(offset);
        configRemoteCancoder(sensorPort, offset);
    }

    public void config(){
        //Move motor configuration
        moveConfig.Audio.BeepOnBoot = true;
        moveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        moveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        moveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        moveConfig.Slot0.kV = Constants.Swerve.MOVE_KF;
        moveConfig.Slot0.kS = 0;
        moveConfig.Slot0.kP = Constants.Swerve.MOVE_KP;
        moveConfig.Slot0.kI = Constants.Swerve.MOVE_KI;
        moveConfig.Slot0.kD = Constants.Swerve.MOVE_KD;
        moveConfig.Voltage.PeakForwardVoltage = 12;
        moveConfig.Voltage.PeakReverseVoltage = -12;
        moveConfig.CurrentLimits.StatorCurrentLimit = 60;
        moveConfig.CurrentLimits.SupplyCurrentLimit = 80;
        moveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        moveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        moveConfigurator.apply(moveConfig);
        moveMotor.setRotorPosition(0);

        //Turn motor configuratoin
        turnConfig.Audio.BeepOnBoot = true;
        turnConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnConfig.Slot0.kV = Constants.Swerve.TURN_KF;
        turnConfig.Slot0.kP = Constants.Swerve.TURN_KP;
        turnConfig.Slot0.kI = Constants.Swerve.TURN_KI;
        turnConfig.Slot0.kD = Constants.Swerve.TURN_KD;
        turnConfig.Voltage.PeakForwardVoltage = 12;
        turnConfig.Voltage.PeakReverseVoltage = -12;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 103;
        turnConfig.MotionMagic.MotionMagicAcceleration = 180;
        turnConfig.MotionMagic.MotionMagicJerk = 0;
        turnConfig.CurrentLimits.StatorCurrentLimit = 60;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 80;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnConfigurator.apply(turnConfig);
        

        sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        sensorConfigurator.apply(sensorConfig);
    }

    public void configRemoteCancoder(int cancoderID, double offset){
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
        turnConfig.Feedback.SensorToMechanismRatio = 1;
        turnConfig.Feedback.RotorToSensorRatio = Constants.Swerve.TURN_GEAR_RATIO;
        turnConfig.Feedback.FeedbackRotorOffset = offset;
        turnConfigurator.apply(turnConfig);
    }

    public void configOffset(double offset){
        sensorConfig.MagnetSensor.MagnetOffset = offset;
        sensorConfigurator.apply(sensorConfig);
        turnMotor.setRotorPosition(NRUnits.Drive.degToNU(sensor.getAbsolutePosition().getValue()*360));
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.movePosition = moveMotor.getPosition().getValue();
        inputs.moveVelocity = moveMotor.getVelocity().getValue();
        inputs.moveVoltage = moveMotor.getSupplyVoltage().getValue();
        inputs.moveStatorCurrent = moveMotor.getStatorCurrent().getValue();
        inputs.moveSupplyCurrent = moveMotor.getSupplyCurrent().getValue();

        inputs.turnAbsolutePosition = sensor.getAbsolutePosition().getValue();
        inputs.turnRotorPosition = turnMotor.getRotorPosition().getValue();
        inputs.turnVelocity = turnMotor.getVelocity().getValue();
        inputs.turnVoltage = turnMotor.getSupplyVoltage().getValue();
        inputs.turnStatorCurrent = turnMotor.getStatorCurrent().getValue();
        inputs.turnSupplyCurrent = turnMotor.getSupplyCurrent().getValue();
        inputs.sensorVelocity = sensor.getVelocity().getValue();
    }

    public void setMoveVelocity(double velocity) {
        moveControl.Velocity = velocity;
        moveMotor.setControl(moveControl);
    }

    public void setTurnPosition(double position) {
        turnControl.Position = position;
        turnMotor.setControl(turnControl);
    }

    public void setTurnVelocity(double velocity){
        turnVelocityControl.Velocity = velocity;
        turnMotor.setControl(turnVelocityControl);
    }

    public void assignTurnPosition(double position) {
        turnMotor.setRotorPosition(position);
    }

    public void setkF(double kF){
        turnConfig.Slot0.kV = kF;
        turnConfigurator.apply(turnConfig);
    }

    public void setkP(double kP){
        turnConfig.Slot0.kP = kP;
        turnConfigurator.apply(turnConfig);
    }
    public void setkD(double kD){
        turnConfig.Slot0.kD = kD;
        turnConfigurator.apply(turnConfig);
    }

}