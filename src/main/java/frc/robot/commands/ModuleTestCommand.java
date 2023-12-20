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
    DoubleLogEntry angleEntry;

    public ModuleTestCommand(){
        double offset = 0;
        mod = Robot.mod;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("kF", 0);
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("Turn Angle", 0);
        
        kF = 0;
        kP = 0;
        kD = 0;

        lastkF = 0;
        lastkP = 0;
        lastkD = 0;

        DataLogManager.start();
        log = DataLogManager.getLog();
        angleEntry = new DoubleLogEntry(log, "Angle");
        mod.setkF(0);
        mod.setkP(0);
        mod.setkD(0);

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
        double turn = SmartDashboard.getNumber("Turn Angle", 0);
        mod.setTurnVelocity(turn);;

        angleEntry.append(mod.getTurnVelocity()); //Output in degrees
    }
}
