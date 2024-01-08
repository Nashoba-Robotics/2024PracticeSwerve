package frc.robot.commands;


import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Module;

public class ModuleTestCommand extends CommandBase{
    Module mod;
    double kP, lastkP;
    double kF, lastkF;
    double kD, lastkD;

    DataLog log;
    DoubleLogEntry velocityEntry;
    DoubleLogEntry targetEntry;

    public ModuleTestCommand(){
        
    }

    @Override
    public void initialize() {
        kF = 0.073; //Mod1: 0.075
        kP = 0.09;  //Mod2: 0.09
        kD = 0;   //Mod3: 0.0001

        lastkF = kF;
        lastkP = kP;
        lastkD = kD;

        DataLogManager.start();
        log = DataLogManager.getLog();
        velocityEntry = new DoubleLogEntry(log, "Velocity");
        targetEntry = new DoubleLogEntry(log, "Target");

        SmartDashboard.putNumber("kF", kF);
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("Velocity", 0);

        mod.setkF(kF);
        mod.setkP(kP);
        mod.setkD(kD);

    }

    @Override
    public void execute() {
        kF = SmartDashboard.getNumber("kF", 0);
        if(kF != lastkF){
            mod.setkF(kF);
            lastkF = kF;
        }
        kP = SmartDashboard.getNumber("kP", 0);
        if(kP != lastkP){
            mod.setkP(kP);
            lastkP = kP;
        }
        kD = SmartDashboard.getNumber("kD", 0);
        if(kD != lastkD){
            mod.setkD(kD);
            lastkD = kD;
        }

        //Input in degrees
        double velocity = SmartDashboard.getNumber("Velocity", 0);
        mod.setMoveVelocity(velocity);

        velocityEntry.append(mod.getMoveVelocity()); //Output in degrees
        targetEntry.append(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        // velocityEntry.finish();
        // targetEntry.finish();
        // log.close();
        mod.setMoveVelocity(0);
    }
}
