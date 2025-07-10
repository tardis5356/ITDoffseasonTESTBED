package org.firstinspires.ftc.teamcode.IMUAutoTest;

import static org.firstinspires.ftc.teamcode.IMUAutoTest.AutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.IMUAutoTest.AutoTrajectories.spinTest1;
import static org.firstinspires.ftc.teamcode.IMUAutoTest.AutoTrajectories.spinTest2;
import static org.firstinspires.ftc.teamcode.IMUAutoTest.AutoTrajectories.spinTest3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Set;


@Autonomous(name = "SpinTest")


public class SpinTest extends OpMode {



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    // vision here that outputs position
    int visionOutputPosition = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;

    private ActionCommand SpinTest1, SpinTest2, SpinTest3;

    private RRSubsystem rrSubsystem;

    //    private ExampleSubsystem robot = ExampleSubsystem.getInstance();
    private boolean commandsScheduled = false;


    private ElapsedTime time_since_start;
    private double loop;

    private MecanumDrive drive;
    static String botState;
    public static final Pose2d StartPos = new Pose2d(0, 0, Math.toRadians(90));

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Removes previous Commands from scheduler
        CommandScheduler.getInstance().reset();


        drive = new MecanumDrive(hardwareMap, StartPos);
        telemetry.addData("Status", "Initialized");
        //add initial position
// this line is needed or you get a Dashboard preview error
        generateTrajectories(new MecanumDrive(hardwareMap, StartPos));



        rrSubsystem = new RRSubsystem(hardwareMap);

        Set<Subsystem> requirements = Set.of(rrSubsystem);

        drive.lazyImu.get().resetYaw();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        botState = "specimen";

        Set<Subsystem> requirements = Set.of(rrSubsystem);
        runtime.reset();

        generateTrajectories(new MecanumDrive(hardwareMap, StartPos));

        SpinTest1 = new ActionCommand(spinTest1, requirements);
        SpinTest2 = new ActionCommand(spinTest2, requirements);
        SpinTest3 = new ActionCommand(spinTest3, requirements);



        time_since_start = new ElapsedTime();


        CommandScheduler.getInstance().schedule(


                new SequentialCommandGroup(
                        SpinTest1,
                        SpinTest2,
                        SpinTest3

                )
        );
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();


        // Note: to access the drive position info, needed to declare a drive = mecanumDrive as private variable at top of this class
        telemetry.addData("In loop Heading", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("IMU Heading", drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.update();
        // Setup a variable for each drive wheel to save power level for telemetry


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
