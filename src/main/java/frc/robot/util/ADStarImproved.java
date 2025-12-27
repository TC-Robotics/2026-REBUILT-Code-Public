package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class ADStarImproved implements Pathfinder {
    @Override
    public boolean isNewPathAvailable() {
        return false;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        return null;
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {

    }
}
