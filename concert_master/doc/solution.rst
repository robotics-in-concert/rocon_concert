Creating a Concert Solution
===========================

Overview
--------

There are two steps to creating a concert solution.

1. Wrapping `concert_master.launch` with your own customised arg settings.
2. A list of configured concert services that you wish to run. 

The Launcher
------------

Wrapping the concert master launcher is simply a matter of setting args in your own
``concert.roslaunch`` and including the ``concert_master/concert_master.launch`` file.
The most oft used arguments to set include:

* **concert_name**, **concert_icon**, **concert_description** : for advertising your concert to clients. 
* **scheduler_type** : usually the default is fine, but sometimes you'll write your own or have specific needs.
* **services** : a list of services to start on the concert, more information below.
* **local_machine_only** : useful for testing with simulated clients on a single pc to avoid the noise of other concerts on the network.

Example
^^^^^^^

An example from `turtle_concert`_:

.. code-block:: xml

    <launch>
        <arg name="turtle_services" default="turtle_concert/turtle.services"/>
        <arg name="concert_name" default="Turtle Concert"/>
        <arg name="scheduler_type" default="compatibility_tree"/>
        <arg name="local_machine_only" default="true"/>  <!-- only invite clients if they are on the same pc -->

        <include file="$(find concert_master)/launch/concert_master.launch">
            <arg name="concert_name" value="$(arg concert_name)"/>
            <arg name="services" value="$(arg turtle_services)"/>
            <arg name="conductor_auto_invite" value="true" />
            <arg name="conductor_local_clients_only" value="$(arg local_machine_only)" />
            <arg name="auto_enable_services" value="true" />
            <arg name="scheduler_type" value="$(arg scheduler_type)"/>
        </include>
    </launch>


.. _`turtle_concert`: http://wiki.ros.org/turtle_concert


Services
--------

In the example launcher above we passed in a service argument of the form:

.. code-block:: xml
  
    <arg name="turtle_services" default="turtle_concert/turtle.services"/>
    ....
        <arg name="services" value="$(arg turtle_services)"/>
    
This argument takes a resource location to a yaml file which specifies the list of
services that serves solution. It includes the resource name of the service definition
file [#f1]_ as well as parameters that override the default service
parameters.

All services have a common set of overrideable parameters:

* *name* : name given to this service that is also used to uniquely identify it
* *priority* : the default priority used for resource requests (see scheduler_msgs.Request.XXX_PRIORITY constants for recommended levels) 
* *description* : a human readable paragraph that describes this service.
* *parameters* : a ros resource name used to lookup service specific parameterisations 

Depending on the service and the service type it may also have its own set of
overrideable parameters (just like a ros node's parameters). These can be configured by
setting them in a yaml file and referencing that via the above *parameters* field.

Example
^^^^^^^

**turtle_concert/turtle.services**

.. code-block:: yaml
   
   - resource_name: concert_service_turtlesim/turtlesim
     overrides:
       parameters: turtle_concert/turtlesim
   - resource_name: turtle_concert/turtle_pond
   - resource_name: concert_service_admin/admin
   - resource_name: concert_service_teleop/teleop

Here the parameters file is used to configure how many and which simulated turtles
should be launched:

.. code-block:: yaml

   # These are all req'd, even if empty
   turtles:
     kobuki:
       rapp_whitelist: [rocon_apps, turtle_concert]
       concert_whitelist: [Turtle Concert, Turtle Teleop Concert, Concert Tutorial]
     guimul:
       rapp_whitelist: [rocon_apps, turtle_concert]
       concert_whitelist: [Turtle Concert, Turtle Teleop Concert, Concert Tutorial] 

Footnotes
^^^^^^^^^

.. [#f1] A specification for the service definition file is provide elsewhere - here we only elaborate on how to enable and parameterise a concert service.

       