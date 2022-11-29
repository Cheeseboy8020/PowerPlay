package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.commands.Coefficients.kD;
import static org.firstinspires.ftc.teamcode.commands.Coefficients.kI;
import static org.firstinspires.ftc.teamcode.commands.Coefficients.kP;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftGoToPos extends CommandBase {
    private PIDFController liftController;
    public static PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD);
    private double kStatic = 0.1; //gravity
    private double tolerance;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public LiftGoToPos(Lift lift, Lift.Positions pos){
        this.lift = lift;
        this.tolerance = 10;
        if(pos.getTargetPosition() == 50 || pos.getTargetPosition() == -50) {
            this.targetPosition = lift.getLift().getCurrentPosition() + pos.getTargetPosition();
        } else {
            this.targetPosition = pos.getTargetPosition();
        }

        liftController = new PIDFController(coefficients, 0, 0, kStatic);
        liftController.setOutputBounds(-0.8, 1);
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        //once
        lift.getLift().setPower(0);
        liftController.reset();
        liftController.setTargetPosition(targetPosition);
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        liftPosition = lift.getLift().getCurrentPosition();
        //Update the lift power with the
        lift.getLift().setPower(liftController.update(liftPosition));
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return Math.abs(liftPosition - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted){
        lift.getLift().setPower(0);
    }

}