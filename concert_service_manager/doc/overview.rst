Overview
========

Component responsible for launching and managing concert services.

ROS APIs
--------

ROS APIs are documented in `ROS Wiki`_.

.. _`ROS Wiki`: http://wiki.ros.org/concert_service_manager

Enable Service Process
----------------------

Concert service can be enabled via *default_auto_enable_services* parameter or ros service call. 
When it receives enable service request, the enabling process is the following.

#. Reload service pool and check if the requested service availble. Also checks if it is already enabled. in service_manager.py
#. Load default service parameters(name, description, priority, and uuid. in service_manager.py
#. Load custom service parameters(params in .parameters). in service_instance.py
#. Launch service instance as subprocess. in service_instance.py
#. Load service interactions. in serviceinstance.py

Service Caching
----------------

Service local caching is designed to separate default service parameters and local configuration. Please check design and usage via github issues.
This feature can be enabled to set *disable_cache* param as false.

* `Github service cache design discussion`_

.. _`Github service cache design discussion`: https://github.com/robotics-in-concert/rocon_concert/issues/254
