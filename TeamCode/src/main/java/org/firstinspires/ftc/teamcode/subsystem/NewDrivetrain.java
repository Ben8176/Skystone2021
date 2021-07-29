package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Vector2d;

public class NewDrivetrain {

    double curr_heading;

    public NewDrivetrain(DcMotorEx m0, DcMotorEx m1, DcMotorEx m2, DcMotorEx m3) {

    }

    /*
    Public methods to drive diffy
     */
    public void driveDiffy(double tra_angle, double rot_angle, double tra_pow, double rot_pow) {
    }

    public void driveDiffy(double tra_angle, double rot_angle) {
    }



    public void update(double heading) {
        curr_heading = heading;
    }
}
