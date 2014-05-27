Quickstart
==========

The following will launch a default concert solution:


.. code-block:: bash

    $ roslaunch concert_master concert_master.launch --screen

A default solution will have the following:

* A `zeroconf_avahi`_ node broadcasting it's presence on the network
* The `rocon gateway components`_ for discovery and communicating with ros robots (on separate masters). 
* The `concert_conductor`_ for inviting and managing connections to ros robots.
* The `concert_service_manager`_ for managing concert services.
* A simple node which advertises introspectable concert information to remocons.
* The default scheduler for allocating robot resources to concert services.
* A concert with *zero* services loaded.

Since there are no services, it won't actually *run* anything, but it is useful for
investigating what are the core parts of the concert. Services can be loaded either via
launcher arg, or by calling the appropriate ros service handles provided by the concert
service manager (note concert services are entire ros subsystems vs the simple
middleware communication mechanism that is a ros service).

.. _`remocons`: http://wiki.ros.org/rocon_remocon
.. _`concert_conductor`: http://wiki.ros.org/concert_conductor
.. _`concert_service_manager`: http://wiki.ros.org/concert_service_manager
.. _`rocon gateway components`: http://wiki.ros.org/rocon_gateway
.. _`zeroconf_avahi`: http://wiki.ros.org/zeroconf_avahi
