Introspection
=============

Ros Api
-------

These are the ros api of the concert conductor node. These are
only of real interest for directly introspecting the state of concert
clients (using the introspecting tools is far easier).

**Published Topics**


 * ``~concert_clients`` (`concert_msgs`_/ConcertClients) : periodic publishing of all invited and uninvited clients on the network.
 * ``~concert_client_changes`` (`concert_msgs`_/ConcertClients) : similar to the previous publisher, but noly published when there is a change, latched.
 * ``~graph`` (`concert_msgs`_/ConductorGraph) : used by introspecting tools and shows information about *all* clients (visible, missing, bad, ...), latched.

**Subscribed Topics**

 * ``~status`` (`rocon_app_manager_msgs`_/Status) : used to keep tabs on what rapps are running and any retrieve any connectivity statistics from the client.

Introspection Tools
-------------------

The ``concert_conductor_graph`` script can be used to introspect the output of the graph publisher and can be run in two modes. By default it
will attempt to call on the `concert_conductor_graph`_ rqt plugin to visualise the graph of all known concert clients. If this is not available, or
it detects no graphical display on your system, it will fall back to pretty printing the output in a convenient to read format.


.. _`concert_conductor_graph`: http://wiki.ros.org/concert_conductor_graph
.. _`concert_msgs`: http://wiki.ros.org/concert_msgs
.. _`rocon_app_manager_msgs`: http://wiki.ros.org/rocon_app_manager_msgs