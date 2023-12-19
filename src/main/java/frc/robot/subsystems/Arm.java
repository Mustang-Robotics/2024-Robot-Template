
//SUMMARY NOTES: Declares the modules relating to the motors. (One Motor)
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class Arm extends ProfiledPIDSubsystem{

    
    public static ShuffleboardTab tab = Shuffleboard.getTab("Arm"); //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    private GenericEntry forward_ks = tab.add("forward_ks", 0).getEntry();
    private GenericEntry forward_kg = tab.add("forward_kg", 0).getEntry();
    private GenericEntry forward_kv = tab.add("forward_kv", 0).getEntry();
    private GenericEntry forward_ka = tab.add("forward_ka", 0).getEntry();
    //private GenericEntry velocity = tab.add("velocity", 720).getEntry();
    //private GenericEntry acceleration = tab.add("acceleration", 720).getEntry();


    private final WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
    private final AnalogPotentiometer pot = new AnalogPotentiometer(0, 280, 0);
    private ArmFeedforward m_feedforward = new ArmFeedforward(forward_ks.getDouble(0), forward_kg.getDouble(0), forward_kv.getDouble(0), forward_ka.getDouble(0));
    
    public Arm(){
        super(
            new ProfiledPIDController(
            1, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                720, 
                720)), 
                0);
        setGoal(90);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint){
        m_feedforward = new ArmFeedforward(forward_ks.getDouble(0), forward_kg.getDouble(0), forward_kv.getDouble(0), forward_ka.getDouble(0));
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        armMotor.setVoltage(output + feedforward);
    }
    @Override
    public double getMeasurement(){
        return pot.get();
    }


    public double ArmAngle() {
        return pot.get();
    }
    public void ArmMovementUp() { // NOTE: Starts the movement of the motor from 0-1. 1 is full, 0 is none.
        armMotor.set(.3);
    }

    public void ArmMovementDown() {
        armMotor.set(0);
    }

}
