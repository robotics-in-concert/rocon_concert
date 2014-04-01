Rocon Service
=============

*Rocon Service* is a high level workflow which `Solution` provides. 

Features
--------

- *Parallelise services* - multiple services inside a solution, like a server pc.
- *Utilise retaskable clients* - request for robots and devices on demand
- *Launch software as needed* - spawn software processes on a concert pc farm.
- *Varied kinds* - support for ros, static link graph, orc and bpel variants.
- *Standardised Interfaces* - standardise service periphery to resource scheduling, human interactions and software node launching.

The last point is important. We aren't likely to have the perfect orchestration technique for composing a service. There probably isn't a best option for all use cases either. In addition, plenty of folk like trying different things in this space, so if possible, let's accomodate this feature.

Specification
-------------

.. code-block:: yaml
    
    name: <STRING>
    description: <STRING>
    author: <STRING>
    priority: <INT>
    launcher_type: <TYPES_IN_concert_msgs.CONCERT_SERVICE.msg>
    launcher: <VARY>
    parameters: <PACKAGE_NAME>/<.parameter>- 
    interactions - <PACKAGE_NAME>/</interactions>

**Note**

* Name, description, priority are required. 
* `(launcher_type && launcher) || interactions` are required 

Supported Launcher Type
+++++++++++++++++++++++

* *ROSLAUNCH* - <PACKAGE_NAME>/<LAUNCH_NAME>
* *BPEL* - Coming Soon 
* *SHADOW* - TODO Add description later

Example
-------

**Admin**

.. code-block:: yaml
    
    name: Admin
    description: Administrative services and tools
    author: DanielStonier
    priority: 1
    interactions: concert_service_admin/admin

**Turtlesim**

.. code-block:: yaml

    name: TurtleSim
    description: Turtlesim service 
    author: Yujin Robot
    priority: 3
    launcher_type: roslaunch
    launcher: concert_service_turtlesim/turtlesim
    parameters: concert_service_turtlesim/turtlesim.parameters 
