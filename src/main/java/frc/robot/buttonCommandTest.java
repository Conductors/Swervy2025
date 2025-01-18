package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class is a test to figure out how Command-based programming works; replace with real subsystems */
public class buttonCommandTest extends SubsystemBase {
    
    public Command buttonTest() {
        //the Command must return itself, completing after running the SysOut
        return this.runOnce(() -> System.out.println("BUTTON PRESSED"));
    }

}
