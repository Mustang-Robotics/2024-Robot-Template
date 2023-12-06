
//SUMMARY NOTES: Declares the modules relating to the motors. (One Motor)
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
    
    public void ArmMovementUp() { // NOTE: Starts the movement of the motor from 0-1. 1 is full, 0 is none.
        armMotor.set(1);
    }

    public void ArmMovementDown() {
        armMotor.set(0);
    }

}
