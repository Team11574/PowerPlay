package incognito.teamcode.testPackageRemoveLater;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.config.variable.VariableType;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import incognito.cog.trajectory.TrajectorySequence;
import incognito.cog.trajectory.TrajectorySequenceBuilder;

public class ConfigurableTrajectorySequenceBuilder {

    TrajectorySequenceBuilder builder;

    private List<ConfigurableTrajectory> configTrajectoryList;

    public ConfigurableTrajectorySequenceBuilder(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this(
                startPose, null,
                baseVelConstraint, baseAccelConstraint,
                baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
        );
    }

    public ConfigurableTrajectorySequenceBuilder(
            Pose2d startPose,
            Double startTangent,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        addConfigurableTrajectory(new ConfigurableTrajectory("start",
                new ConfigurablePose(startPose), new ConfigurableDouble(startTangent),
                baseVelConstraint, baseAccelConstraint,
                baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
        ));
    }

    @NonNull
    @Override
    public String toString() {
        return configTrajectoryList.toString();
    }

    public CustomVariable getConfigurableVariables() {
        CustomVariable customVariable = new CustomVariable();
        for (ConfigurableTrajectory configTrajectory : configTrajectoryList) {
            customVariable.putVariable(configTrajectory.getType(), configTrajectory.getConfigurableElement());
        }
        return customVariable;
    }

    private ConfigurableTrajectorySequenceBuilder addConfigurableTrajectory(ConfigurableTrajectory configTrajectory) {
        configTrajectoryList.add(configTrajectory);
        return this;
    }

    public TrajectorySequence build() {
        createTrajectoriesFromConfiguration();
        return builder.build();
    }

    private void createTrajectoriesFromConfiguration() {
        for (ConfigurableTrajectory configTrajectory : configTrajectoryList) {
            addTrajectorySuper(configTrajectory);
        }
    }

    private void addTrajectorySuper(ConfigurableTrajectory configTrajectory) {
        switch (configTrajectory.getType()) {
            case "start":
                builder = new TrajectorySequenceBuilder(
                        (Pose2d) configTrajectory.getItem(0),
                        (Double) configTrajectory.getItem(1),
                        (TrajectoryVelocityConstraint) configTrajectory.getItem(2),
                        (TrajectoryAccelerationConstraint) configTrajectory.getItem(3),
                        (Double) configTrajectory.getItem(4),
                        (Double) configTrajectory.getItem(5)
                );
            case "lineTo":
                if (configTrajectory.size() == 1) {
                    builder.lineTo(
                            (Vector2d) configTrajectory.getItem(0)
                    );
                } else {
                    builder.lineTo(
                            (Vector2d) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "lineToConstantHeading":
                if (configTrajectory.size() == 1) {
                    builder.lineToConstantHeading(
                            (Vector2d) configTrajectory.getItem(0)
                    );
                } else {
                    builder.lineToConstantHeading(
                            (Vector2d) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "lineToLinearHeading":
                if (configTrajectory.size() == 1) {
                    builder.lineToLinearHeading(
                            (Pose2d) configTrajectory.getItem(0)
                    );
                } else {
                    builder.lineToLinearHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "lineToSplineHeading":
                if (configTrajectory.size() == 1) {
                    builder.lineToSplineHeading(
                            (Pose2d) configTrajectory.getItem(0)
                    );
                } else {
                    builder.lineToSplineHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "strafeTo":
                if (configTrajectory.size() == 1) {
                    builder.strafeTo(
                            (Vector2d) configTrajectory.getItem(0)
                    );
                } else {
                    builder.strafeTo(
                            (Vector2d) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "forward":
                if (configTrajectory.size() == 1) {
                    builder.forward(
                            (double) configTrajectory.getItem(0)
                    );
                } else {
                    builder.forward(
                            (double) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "back":
                if (configTrajectory.size() == 1) {
                    builder.back(
                            (double) configTrajectory.getItem(0)
                    );
                } else {
                    builder.back(
                            (double) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "strafeLeft":
                if (configTrajectory.size() == 1) {
                    builder.strafeLeft(
                            (double) configTrajectory.getItem(0)
                    );
                } else {
                    builder.strafeLeft(
                            (double) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "strafeRight":
                if (configTrajectory.size() == 1) {
                    builder.strafeRight(
                            (double) configTrajectory.getItem(0)
                    );
                } else {
                    builder.strafeRight(
                            (double) configTrajectory.getItem(0),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "splineTo":
                if (configTrajectory.size() == 2) {
                    builder.splineTo(
                            (Vector2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1)
                    );
                } else {
                    builder.splineTo(
                            (Vector2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
            case "splineToConstantHeading":
                if (configTrajectory.size() == 2) {
                    builder.splineToConstantHeading(
                            (Vector2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1)
                    );
                } else {
                    builder.splineToConstantHeading(
                            (Vector2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "splineToLinearHeading":
                if (configTrajectory.size() == 2) {
                    builder.splineToLinearHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1)
                    );
                } else {
                    builder.splineToLinearHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            case "splineToSplineHeading":
                if (configTrajectory.size() == 2) {
                    builder.splineToSplineHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1)
                    );
                } else {
                    builder.splineToSplineHeading(
                            (Pose2d) configTrajectory.getItem(0),
                            (Double) configTrajectory.getItem(1),
                            (TrajectoryVelocityConstraint) configTrajectory.getItem(1),
                            (TrajectoryAccelerationConstraint) configTrajectory.getItem(2)
                    );
                }
                break;
            default:
                throw new RuntimeException("Invalid trajectory type");
        }
    }

    public ConfigurableTrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineTo",
                        new ConfigurableVector(endPosition)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineTo",
                        new ConfigurableVector(endPosition),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineTo(endPosition, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToConstantHeading(Vector2d endPosition) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToConstantHeading",
                        new ConfigurableVector(endPosition)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToConstantHeading(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToConstantHeading",
                        new ConfigurableVector(endPosition),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToConstantHeading(endPosition, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToLinearHeading",
                        new ConfigurablePose(endPose)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToLinearHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToLinearHeading",
                        new ConfigurablePose(endPose),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(endPose, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToSplineHeading",
                        new ConfigurablePose(endPose)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder lineToSplineHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "lineToSplineHeading",
                        new ConfigurablePose(endPose),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(endPose, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeTo(Vector2d endPosition) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeTo",
                        new ConfigurableVector(endPosition)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeTo",
                        new ConfigurableVector(endPosition),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeTo(endPosition, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder forward(double distance) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "forward",
                        new ConfigurableDouble(distance)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.forward(distance, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder forward(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "forward",
                        new ConfigurableDouble(distance),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.forward(distance, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder back(double distance) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "back",
                        new ConfigurableDouble(distance)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.back(distance, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder back(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "back",
                        new ConfigurableDouble(distance),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.back(distance, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeLeft(double distance) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeLeft",
                        new ConfigurableDouble(distance)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeLeft(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeLeft",
                        new ConfigurableDouble(distance),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeRight(double distance) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeRight",
                        new ConfigurableDouble(distance)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder strafeRight(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "strafeRight",
                        new ConfigurableDouble(distance),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineTo",
                        new ConfigurableVector(endPosition),
                        new ConfigurableDouble(endHeading)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineTo(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineTo",
                        new ConfigurableVector(endPosition),
                        new ConfigurableDouble(endHeading),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition, endHeading, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToConstantHeading(Vector2d endPosition, double endHeading) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToConstantHeading",
                        new ConfigurableVector(endPosition),
                        new ConfigurableDouble(endHeading)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToConstantHeading(
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToConstantHeading",
                        new ConfigurableVector(endPosition),
                        new ConfigurableDouble(endHeading),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endHeading) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToLinearHeading",
                        new ConfigurablePose(endPose),
                        new ConfigurableDouble(endHeading)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToLinearHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToLinearHeading",
                        new ConfigurablePose(endPose),
                        new ConfigurableDouble(endHeading),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endHeading) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToSplineHeading",
                        new ConfigurablePose(endPose),
                        new ConfigurableDouble(endHeading)
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, currentVelConstraint, currentAccelConstraint));
    }

    public ConfigurableTrajectorySequenceBuilder splineToSplineHeading(
            Pose2d endPose,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addConfigurableTrajectory(
                new ConfigurableTrajectory(
                        "splineToSplineHeading",
                        new ConfigurablePose(endPose),
                        new ConfigurableDouble(endHeading),
                        velConstraint,
                        accelConstraint
                )
        );
        //return addPath(() -> currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, velConstraint, accelConstraint));
    }

    private abstract class ConfigurableElement {
        public abstract Object getValue();
    }

    private class ConfigurablePose extends ConfigurableElement {
        ConfigurableDouble x;
        ConfigurableDouble y;
        ConfigurableDouble heading;
        public ConfigurablePose(Pose2d pose) {
            x = new ConfigurableDouble(pose.getX());
            y = new ConfigurableDouble(pose.getY());
            heading = new ConfigurableDouble(pose.getHeading());
        }
        public Object getValue() {
            return new Pose2d((Double) getX().getValue(), (Double) getY().getValue(), (Double) getHeading().getValue());
        }
        public ConfigurableDouble getX() {
            return x;
        }
        public ConfigurableDouble getY() {
            return y;
        }
        public ConfigurableDouble getHeading() {
            return heading;
        }

        @NonNull
        @Override
        public String toString() {
            return getValue().toString();
        }
    }

    private class ConfigurableVector extends ConfigurableElement {
        ConfigurableDouble x;
        ConfigurableDouble y;
        public ConfigurableVector(Vector2d vector2d) {
            x = new ConfigurableDouble(vector2d.getX());
            y = new ConfigurableDouble(vector2d.getY());
        }
        public Object getValue() {
            return new Vector2d((Double) getX().getValue(), (Double) getY().getValue());
        }
        public ConfigurableDouble getX() {
            return x;
        }
        public ConfigurableDouble getY() {
            return y;
        }
        @NonNull
        @Override
        public String toString() {
            return getValue().toString();
        }
    }

    private class ConfigurableDouble extends ConfigurableElement implements ValueProvider<Double> {
        double val;
        public ConfigurableDouble(double input) {
            this.val = input;
        }
        public Object getValue() {
            return val;
        }
        public Double get() {
            return val;
        }
        public void set(Double val) {
            this.val = val;
        }
        @NonNull
        @Override
        public String toString() {
            return getValue().toString();
        }
    }

    private class ConfigurableTrajectory {
        //ConfigurableElement element;
        String type;
        List<Object> args;

        public ConfigurableTrajectory(String type, Object... args) {
            //this.element = element;
            this.type = type;
            this.args = Arrays.asList(args);
        }

        @NonNull
        @Override
        public String toString() {
            return type + ": " + args.toString();
        }

        public String getType() {
            return type;
        }

        public Object getItem(int index) {
            Object item = args.get(index);
            if (item instanceof ConfigurableElement) {
                return ((ConfigurableElement) item).getValue();
            } else {
                return item;
            }
        }

        public int size() {
            return args.size();
        }

        public CustomVariable getConfigurableElement() {
            CustomVariable super_cv = new CustomVariable();
            for (Object o : args) {
                if (o instanceof ConfigurablePose) {
                    CustomVariable cv = new CustomVariable();
                    cv.putVariable("x",
                            new BasicVariable<>(
                                    VariableType.fromClass(Double.class),
                                    ((ConfigurablePose) o).getX()
                            )
                    );
                    cv.putVariable("y",
                            new BasicVariable<>(
                                    VariableType.fromClass(Double.class),
                                    ((ConfigurablePose) o).getY()
                            )
                    );
                    cv.putVariable("heading",
                            new BasicVariable<>(
                                    VariableType.fromClass(Double.class),
                                    ((ConfigurablePose) o).getHeading()
                            )
                    );
                    super_cv.putVariable("pose", cv);
                }
                if (o instanceof ConfigurableVector) {
                    CustomVariable cv = new CustomVariable();
                    cv.putVariable("x",
                            new BasicVariable<>(
                                    VariableType.fromClass(Double.class),
                                    ((ConfigurableVector) o).getX()
                            )
                    );
                    cv.putVariable("y",
                            new BasicVariable<>(
                                    VariableType.fromClass(Double.class),
                                    ((ConfigurableVector) o).getY()
                            )
                    );
                    super_cv.putVariable("vector", cv);
                }
                if (o instanceof ConfigurableDouble) {
                    super_cv.putVariable("value", new BasicVariable<>(
                            VariableType.fromClass(Double.class),
                            (ConfigurableDouble) o
                    ));
                }
            }
            return super_cv;
        }
    }
}
