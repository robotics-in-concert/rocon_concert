
## Launchers

**role_manager.launch**

This is the generic launcher for starting a role manager in the concert. Does not need any other infrastructure to run.

```
> roslaunch concert_roles role_manager.launch
```

## Publishers

**/concert/icon** [rocon_std_msgs/Icon]

Publishes (latched) an icon that can be used to represent the concert. The role manager is doing this since nobody else in the concert currently is and it is the only one with a reason to do so (the remocons need an icon to represent the concert like the pairing robot remocon does for robots).

**/concert/roles** [concert_msgs/Roles]

A list of strings representing the roles managed by the role manager and configured by the services. Remocons should use this when a 'concert' gets selected.

## Services

**/concert/get_roles_and_apps** [concert_msgs/GetRolesAndApps]

Send this service a role (or list of roles) and your platform info and the role manager will filter it's role-app table to find the list of apps for each specified role that will run on that platform. This is used by the remocon to populate the app lists for their users once a role is selected.

## Scripts

**concert_roles_and_apps**

Convenience script that calls and pretty prints the output of `/concert/get_roles_and_apps` with no filtering (i.e. retrieves the whole role-app table).