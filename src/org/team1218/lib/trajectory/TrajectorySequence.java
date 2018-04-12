package org.team1218.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team1218.lib.trajectory.io.PathManager;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;

public class TrajectorySequence {
	protected List<Waypoint> waypoints = new ArrayList<Waypoint>();
	
	public void addWaypoint(Waypoint waypoint) {
		waypoints.add(waypoint);
	}
	
	public Path[] makePaths(TrajectoryGenerator.Config config, double effectiveTrackWidth, String name) {
		Path paths[] = new Path[waypoints.size()-1];
		for(int i = 0; i < waypoints.size()-1; i++) {
			WaypointSequence ws = new WaypointSequence(2);
			ws.addWaypoint(waypoints.get(i));
			ws.addWaypoint(waypoints.get(i+1));
			paths[i] = PathManager.getPath(ws, config, effectiveTrackWidth, name + i);
		}
		return paths;
	}
}
