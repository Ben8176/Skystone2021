package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;

@Config
@TeleOp(name = "Driver Control")
public class SkystoneDriverControl extends LinearOpMode {

//    //save encoder values to local instance variables
    public double m_m0Encoder = Drivetrain.m0Encoder;
    public double m_m1Encoder = Drivetrain.m1Encoder;
    public double m_m2Encoder = Drivetrain.m2Encoder;
    public double m_m3Encoder = Drivetrain.m3Encoder;

    // Set the start pose of driver to be the auto final pose
    public Pose2d startPose = Robot.autoFinalPose;

    @Override
    public void runOpMode() {

        telemetry.addData("Robot Initialized: ", "FALSE");
        telemetry.update();

        Robot robot = new Robot(hardwareMap, startPose, telemetry);

        robot.initAll();

        //set motor encoder values from auto
        robot.drive.setEncoderOffsets(m_m0Encoder, m_m1Encoder, m_m2Encoder, m_m3Encoder);

        telemetry.addData("Robot Initialized: ", "TRUE");
        telemetry.update();

        //=======================================
        waitForStart();
        //=======================================

        while (opModeIsActive()) {

            //-----------------------
            //update robot pose and data from REV Hubs
            robot.update();

//            robot.ndrive.driveDiffy(new Vector2d(0,1), 0);

//            telemetry.addData("Target Vector X: ", robot.ndrive.rmod.getTargetVec().getX());
//            telemetry.addData("Target Vector Y: ", robot.ndrive.rmod.getTargetVec().getY());
//            telemetry.addData("Translation Comp: ", robot.ndrive.rmod.getPowerVec().getX());
//            telemetry.addData("Rotation Comp: ", robot.ndrive.rmod.getPowerVec().getY());
            telemetry.addData("Module Orientation: ", robot.ndrive.rmod.getModuleOrientation());
            telemetry.update();
        }
    }
}
