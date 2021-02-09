package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

// The field:
//
//                   Test Image
// (0,0)   {B}
//     ---------------------------------------
//     |                                     |
//     |                                     |
//     |                                     |
//     |                  |                  |
//    z|                 -+-                 |
//     |                  |                  |
// {ba}|                                     |
//     |     |   |                           |
//     |  [.]|   |                           |
//     ---------------------------------------
//
// [.] = bot starting
// {B} is about 3ft from the left or around 914mm.
//

public class VisionNavigator {
    public static final long NEVER_SAW = -1;
    private static final String TAG = "VisionNavigator";

    public static final String LABEL_BLUE_TARGET = "BlueTarget";
    public static final String LABEL_BLUE_ALLIANCE = "BlueAlliance";

    /**
     * We use units of mm here because that's the recommended units of measurement for the
     * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
     * <ImageTarget name="stones" size="247 173"/>
     * You don't *have to* use mm here, but the units here and the units used in the XML
     * target configuration files *must* correspond for the math to work out correctly.
     */
    private static final float mmPerInch = 25.4f;
    private static final float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    private static final float mmFTCFieldWidth  = (12*8 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    private static final float mmFTCFieldHeight  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    private static final LowPassFilter lpfX = new LowPassFilter(1000);
    private static final LowPassFilter lpfY = new LowPassFilter(1000);
    private static final LowPassFilter lpfZ = new LowPassFilter(1000);

    private VuforiaTrackables vuforiaUltimateGoalTrackables;
    private List<VuforiaTrackable> trackables;
    private final HashMap<String, VuforiaTrackable> trackableMap = new HashMap<>();
    private final HashMap<String, Long> lastTimeSeenObject = new HashMap<>();

    private OpenGLMatrix lastComputedLocation;
    private long lastComputedLocationTimestamp;

    public void initialize(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        vuforiaUltimateGoalTrackables = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTarget  = vuforiaUltimateGoalTrackables.get(0);
        blueTarget.setName(LABEL_BLUE_TARGET);

        VuforiaTrackable blueAlliance = vuforiaUltimateGoalTrackables.get(3);
        blueAlliance.setName(LABEL_BLUE_ALLIANCE);

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        trackables = new ArrayList<VuforiaTrackable>();
        trackables.add(blueTarget);
        trackables.add(blueAlliance);

//        trackables.addAll(vuforiaUltimateGoalTrackables);

        for (VuforiaTrackable trackable : trackables) {
            trackableMap.put(trackable.getName(), trackable);
        }

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To Blue Target on the back wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it 90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */
        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
            /* Then we translate the target off to the BLUE WALL. Our translation here
            is a negative translation in X.*/
            .translation((3 * 12 * mmPerInch), 0, 0)
            .multiplied(Orientation.getRotationMatrix(
                /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                AxesReference.EXTRINSIC, AxesOrder.XZX,
                AngleUnit.DEGREES, 0, 0, 0));
        blueTarget.setLocation(blueTargetLocationOnField);
        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));

        /*
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Finally, we translate it along the Y axis towards the blue audience wall.
         */
        OpenGLMatrix blueAllianceLocationOnField = OpenGLMatrix
            /* Then we translate the target off to the Blue Audience wall.
            Our translation here is a positive translation in Y.*/
            .translation(0, 0, 0)
            .multiplied(Orientation.getRotationMatrix(
                /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                AxesReference.EXTRINSIC, AxesOrder.XZX,
                AngleUnit.DEGREES, 0, 0, 0));
        blueAlliance.setLocation(blueAllianceLocationOnField);
        RobotLog.ii(TAG, "Blue Alliance=%s", format(blueAllianceLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(0, 0,0)
            .multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.YZY,
                AngleUnit.DEGREES, 0, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);
        ((VuforiaTrackableDefaultListener)blueAlliance.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);
    }

    public void activate() {
        if (vuforiaUltimateGoalTrackables == null) {
            throw new RuntimeException("VisionNavigator is not initialized");
        }

        vuforiaUltimateGoalTrackables.activate();
    }

    public void shutdown() {
        if (vuforiaUltimateGoalTrackables != null) {
            vuforiaUltimateGoalTrackables.deactivate();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void loop() {
        for (VuforiaTrackable trackable : trackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (isCurrentlyVisible(trackable.getName())) {
                lastTimeSeenObject.put(trackable.getName(), System.currentTimeMillis());
            }

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastComputedLocation = robotLocationTransform;
                lastComputedLocationTimestamp = System.currentTimeMillis();

                VectorF translation = lastComputedLocation.getTranslation();

                float[] locationData = translation.getData();
                float x = locationData[0];
                float y = locationData[1];
                float z = locationData[2];

                lpfX.addSample(x, lastComputedLocationTimestamp);
                lpfY.addSample(y, lastComputedLocationTimestamp);
                lpfZ.addSample(z, lastComputedLocationTimestamp);
            }
        }
    }

    public boolean isCurrentlyVisible(String objectLabel) {
        VuforiaTrackable vuforiaTrackable = trackableMap.get(objectLabel);
        if (vuforiaTrackable != null) {
            return ((VuforiaTrackableDefaultListener)vuforiaTrackable.getListener()).isVisible();
        }

        return false;
    }

    public OpenGLMatrix getLastComputedLocation() {
        return lastComputedLocation;
    }

    public long getLastTimeObjectSeen(String objectLabel) {
        return lastTimeSeenObject.getOrDefault(objectLabel, NEVER_SAW);
    }

    public double[] getLastComputedLocationFiltered() {
        OpenGLMatrix lastComputedLocation = getLastComputedLocation();
        if (lastComputedLocation == null) {
            return null;
        }

        return new double[] {
            lpfX.getValue(),
            lpfY.getValue(),
            lpfZ.getValue()
        };
    }
}
