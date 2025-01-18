package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class is a test to figure out how Command-based programming works; replace with real subsystems */
public class buttonCommandTest extends SubsystemBase {
    
    public Command buttonTest() {
        //the Command must return itself, completing after running the SysOut
        System.out.println("buttonTest called");
        return this.runOnce(() -> System.out.println("BUTTON PRESSED"));
    }

    public Command buttonTest2() {
        return Commands.sequence(
            new InstantCommand(() -> System.out.println("buttonTest2 called"))
        );
    }

    public void buttonTest3() {
        System.out.print("buttonTest3 called");
    }

}
