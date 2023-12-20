package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.util.SwerveState;

public class Module {

    public int modIndex;
 
    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private double movePosition;
    private double lastMovePosition;

    public Module(int modIndex, int movePort, int turnPort, int sensorPort, double offset, double AFF) {
        this.modIndex = modIndex;
        multiplier = 1;
        movePosition = 0;
        lastMovePosition = 0;
        io = new ModuleIOTalonFX(movePort, turnPort, sensorPort, offset, AFF);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + modIndex, inputs);
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }

    public void set(SwerveModuleState state){
        set(NRUnits.Drive.toPercentOutput(state.speedMetersPerSecond), state.angle.getRadians());
    }
    
    //move input in percent, Turn input in radians
    public void set(double move, double turn){
        turn *= 360/Constants.TAU;
        setDeg(move, turn);
    }

    public void set(double move, double turn, boolean optimizeTurn){
        turn *= 360/Constants.TAU;
        setDeg(move, turn, optimizeTurn);
    }
    
    //Move input in percent, Turn input in degrees
    public void setDeg(double move, double turn) {
        if(move == 0){
            io.setMoveVelocity(0);
            return;
        }
        double currentPos =  inputs.turnRotorPosition;
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        io.setTurnPosition(nextPos);
        io.setMoveVelocity(move * Constants.Swerve.MAX_NATIVE_VELOCITY * multiplier);
    }

    public void setDeg(double move, double turn, boolean optimizeTurn) {
        if(move == 0 && optimizeTurn){
            io.setMoveVelocity(0);
            return;
        }
        double currentPos =  inputs.turnRotorPosition;
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        io.setTurnPosition(nextPos);
        io.setMoveVelocity(move * Constants.Swerve.MAX_NATIVE_VELOCITY * multiplier);
    }

    //radian input
    public void turn(double turn){
        turn *= 360/Constants.TAU;
        double currentPos =  inputs.turnRotorPosition;
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        io.setTurnPosition(nextPos);
    }

    // MPS, Rotation 2D
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            NRUnits.Drive.NUToM(movePosition),
            Rotation2d.fromRadians(
                multiplier == 1 ?
                NRUnits.constrainRad(getAbsAngle()+Constants.TAU/2):
                getAbsAngle())
        );
    }

    private int multiplier;

    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            // moveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            // moveConfigurator.apply(moveConfig);
            multiplier = -1;
            return potAngles[0];
        }
        else{ //If we want to go to the opposite of the desired angle, we have to tell the motor to move "backwards"
            // moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            // moveConfigurator.apply(moveConfig);
            multiplier = 1;
            return potAngles[1];
        } 
    }

    // Find the two angles we could potentially go to
    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = NRUnits.constrainDeg(angle);

        //Figure out the opposite angle
        double oppositeAngle = angle + 180;

        //Constrain the opposite angle
        oppositeAngle = NRUnits.constrainDeg(oppositeAngle);

        //Put them into a size 2 array
        double[] angles = {angle, oppositeAngle};

        return angles;
    }

    public double findAngleChange(double turn, double lastTurn){
        double distance = turn - lastTurn;
        //double sign = Math.signum(distance);   //Either 1 or -1 -> represents positive or negative

        if(Math.abs(turn - (lastTurn + 360)) < Math.abs(distance)){
            // If this is true, it means that lastTurn is in the negatives and is trying to reach a positive, meaning that it must move positive
            distance = turn - (lastTurn + 360);
            //sign = +1;
        }

        if(Math.abs(turn+360 - (lastTurn)) < Math.abs(distance)){
            // If this is true, it means that turn is in the negatives and lastTurn is trying to reach a negative, meaning that you must move negative 
            distance = turn+360 - lastTurn;
            //sign = -1;
        }

        return distance;
    }

    public double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+360 - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+360)));

        return distance;
    }

    //returns angle in radians
    public double getAngle(){
        return NRUnits.Drive.NUToRad(inputs.turnRotorPosition * 360);
    }

    //returns CANCoder angle in radians
    public double getAbsAngle(){
        return inputs.turnAbsolutePosition * Constants.TAU;
    }

    public double getTurnPosition() {
        return inputs.turnRotorPosition;
    }

    public double getTurnAngle(){
        return NRUnits.constrainRad(getTurnPosition() * Constants.TAU);
    }

    public double getTurnVelocity(){
        return inputs.turnVelocity;
    }
    // RADIANS
    public boolean atTargetAngle(double targetAngle){
        double deadzone = Constants.TAU/12;
        return Math.abs(getTurnAngle() - targetAngle) < deadzone;
    }

    public void setTurnMotor(double position) {
        io.setTurnPosition(position);
    }

    public double getMovePosition() {
        return movePosition;
    }

    public double getMoveVelocity() {
        return inputs.moveVelocity;
    }

    public void setMoveVelocity(double move) {
        io.setMoveVelocity(move);
    }

    public void updateMovePosition() {
        double temp = Math.abs(inputs.movePosition);
        movePosition += Math.abs(temp - lastMovePosition);
        lastMovePosition = temp;
    }
    
    public SwerveModuleState getSwerveState() {
        return new SwerveModuleState(
                NRUnits.Drive.NUToMPS(inputs.moveVelocity),
                // Rotation2d.fromRadians(NRUnits.Drive.NUToRad(turnMotor.getSelectedSensorPosition()))
                Rotation2d.fromRadians(getAbsAngle())
            );
    }

    public double getMoveSensorNU() {
        return inputs.movePosition;
    }

    public void resetTurnToAbsolute() {
        io.setTurnPosition(NRUnits.Drive.degToNU(NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(inputs.turnRotorPosition))));
        io.assignTurnPosition(NRUnits.Drive.degToNU(inputs.turnAbsolutePosition*360));
    }

    public void setkF(double kF){
        io.setkF(kF);
    }
    public void setkP(double kP){
        io.setkP(kP);
    }
    public void setkD(double kD){
        io.setkD(kD);
    }

    public void setDumbTurn(double turn){
        io.setTurnPosition(turn);
    }
    public void setTurnVelocity(double velocity){
        io.setTurnVelocity(velocity);
    }
    public double getSensorSpeed(){
        return inputs.sensorVelocity;
    }
}