ROS API
=======

These are the specifications for the ros api of the concert_software_farm node.

Published Topics
----------------

 * ``~list`` (`concert_msgs`_/SoftwareProfiles) : a list of available software in the system, latched
 * ``~status`` (`concert_msgs`_/SoftwareInstances) : informs the current status of software farm like which software are running and who users are, and the paramter configuration. latched.

Services
--------

* ``~allocate`` (`concert_msgs`_/AllocateSoftware) : used by client to allocate software to user.

.. _`concert_msgs`: http://wiki.ros.org/concert_msgs
