package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.RevBulkData;

public class NewDrivetrain {

    double curr_heading;
    public DriveModule lmod, rmod;
    RevBulkData edata, cdata;

    public NewDrivetrain(DcMotorEx m0, DcMotorEx m1, DcMotorEx m2, DcMotorEx m3) {
        lmod = new DriveModule(m0, m1);
        rmod = new DriveModule(m2, m3);
    }

    /*
    Public methods to drive diffy
    transvec: < field point >
    rotvec: < rot_angle, rot_pow >
     */
    public void driveDiffy(Vector2d transVec) {
        //adjust the target angle for the modules by accounting for robot heading
        double desiredModuleAngle = transVec.rotateBy(-curr_heading).getAngle();
    }

    public void driveDiffy(Vector2d transVec, double rotMagnitude) {
//        lmod.updateTarget(transVec, rotMagnitude);
        rmod.updateTarget(transVec, rotMagnitude);
    }



    public void update(double heading, RevBulkData cdata, RevBulkData edata) {
        curr_heading = heading;
        this.cdata = cdata;
        this.edata = edata;

        //TODO: update these to match config motor #
        double m0Enc = cdata.getMotorCurrentPosition(0);
        double m1Enc = cdata.getMotorCurrentPosition(1);
        lmod.update(m0Enc, m1Enc, curr_heading);
        double m2Enc = edata.getMotorCurrentPosition(0);
        double m3Enc = edata.getMotorCurrentPosition(1);
        rmod.update(m2Enc, m3Enc, curr_heading);
    }
}
