package frc.robot;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.utilities.F310Controller;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private JoystickButton brakeButton, liftButton, debugButton, suckButton, spitButton;
    private F310Controller driveStick, manipulatorStick;

    

    public OI() {
        driveStick = new F310Controller(0);
        brakeButton = new JoystickButton(driveStick, 2);
        brakeButton.whileActive(new Brakes());
        liftButton = new JoystickButton(driveStick, 10);
        liftButton.whileActive(new Lifter());
        debugButton = new JoystickButton(driveStick, 3);
        debugButton.toggleWhenPressed(new DebugOutput());
        manipulatorStick = new F310Controller(1);
        suckButton = new JoystickButton(manipulatorStick, 3);
        suckButton.whileActive(new SuckBall());
        spitButton = new JoystickButton(manipulatorStick, 4);
        spitButton.whileActive(new SpitBall());
        SmartDashboard.putData("ResetEncoders", new ResetEncoders());
        SmartDashboard.putData("ResetGyro", new ResetGyro());
    }

    public F310Controller getDriveStick() {
        return driveStick;
    }

    public F310Controller getManipulatorStick() {
        return manipulatorStick;
    }

    public JoystickButton getBrakeButton() {
        return this.brakeButton;
    }

    public void setBrakeButton(JoystickButton brakeButton) {
        this.brakeButton = brakeButton;
    }

    public JoystickButton getLiftButton() {
        return this.liftButton;
    }

    public void setLiftButton(JoystickButton liftButton) {
        this.liftButton = liftButton;
    }

    public JoystickButton getDebugButton() {
        return this.debugButton;
    }

    public void setDebugButton(JoystickButton debugButton) {
        this.debugButton = debugButton;
    }

    public JoystickButton getSuckButton() {
        return this.suckButton;
    }

    public void setSuckButton(JoystickButton suckButton) {
        this.suckButton = suckButton;
    }

    public JoystickButton getSpitButton() {
        return this.spitButton;
    }

    public void setSpitButton(JoystickButton spitButton) {
        this.spitButton = spitButton;
    }
    public void setDriveStick(F310Controller driveStick) {
        this.driveStick = driveStick;
    }
    public void setManipulatorStick(F310Controller manipulatorStick) {
        this.manipulatorStick = manipulatorStick;
    }

}
