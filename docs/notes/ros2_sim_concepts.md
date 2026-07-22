# General ROS2 + Simulation Concepts (for future projects, not just Percept-Nav)

## use_sim_time — why simulated projects need a shared clock

### The problem
Every ROS2 message carries a timestamp. Normally nodes use the real
system wall-clock. But in simulation, Gazebo's internal simulated time
can run faster or slower than real wall-clock time (e.g. if the machine
is under load and rendering slowly). If some nodes use wall-clock time
and Gazebo's sensors use simulated time, timestamps between them drift
apart and no longer agree on "when" anything happened.

### Why it matters
Anything that compares timestamps across sources -- time-synchronized
sensor fusion (message_filters.ApproximateTimeSynchronizer), SLAM scan
matching, TF lookups -- silently breaks or produces garbage if the
clocks disagree, even though no error is thrown.

### The fix
Pass `use_sim_time:=true` to every node that needs to agree with
Gazebo's clock. This makes the node subscribe to Gazebo's `/clock`
topic and use that as its time source instead of the real system clock.

### The general principle (applies beyond ROS2/Gazebo too)
Any time multiple components are simulated, distributed, or running at
different real/virtual speeds, they need to agree on a single shared
clock source before any of their timing-dependent behavior (fusion,
synchronization, ordering) can be trusted. This is a general distributed-
systems concept, not a ROS2-specific quirk -- worth recognizing in any
future project involving multiple time-dependent components.

### How to check if it's set correctly
    ros2 param get /<node_name> use_sim_time

Should return "true" for any node running alongside Gazebo sim.
