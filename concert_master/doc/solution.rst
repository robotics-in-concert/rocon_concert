Solution
========

Installation
------------

TODO 


Solution Bring-up
-----------------

.. code-block:: bash

    $ roslaunch concert_master concert_master.launch --screen


Preruntime Solution Configuration
---------------------------------

*concert_master.launch* includes configurable arguements

.. code-block:: xml
  
    <!-- ******************************* Arguments ******************************* -->
    <!-- Concert -->
    <arg name="concert_name" default="Pirate Concert"/>
    <arg name="concert_icon" default="concert_master/rocon_text.png"/>
    <arg name="concert_description" default="Pirates in concert."/>
    <!-- Gateways -->
    <arg name="hub_port" default="6380"/>
    <arg name="hub_uri" default="http://localhost:6380"/>
    <arg name="gateway_watch_loop_period" default="2"/> <!-- Polling period for multimaster advertising/flipping -->
    <arg name="gateway_disable_uuids" default="false"/> <!-- manage unique naming of multiple gateways yourself -->
    <arg name="gateway_network_interface" default="$(optenv GATEWAY_NETWORK_INTERFACE)"/>/>  <!-- If you have multiple n
    <!-- Conductor -->
    <arg name="conductor_auto_invite" default="true"/>
    <arg name="conductor_local_clients_only" default="false"/>
    <!-- Service Manager -->
    <arg name="services" default=""/> <!-- service list resource location. e.g. concert_tutorial/tutorial.services --> 
    <arg name="auto_enable_services" default=""/> <!-- autoenable services, e.g. true/false -->
    <!-- Scheduler -->
    <arg name="scheduler_type" default="demo"/>  <!-- options are demo, compatibility_tree -->



Solution Service Configuration
------------------------------

*services* argument takes a resource location to specify list of services that serves solution. It includes name of services as well as parameters to override the default service parameters

**Note that uuid is not overridable parameter** 


Example
^^^^^^^

**concert_tutorial/tutorial.serivces**

.. code-block:: yaml
  
    - name: rocon_service_turtlesim/turtlesim
      override:
        name: My 'TurtleSim'
        priority: 15
        turtle_color: green
    - name: rocon_service_admin/admin
      override:
        description: admin is useful
    - name: turtle_concert/turtle_pond
    - name: chatter_concert/chatter
