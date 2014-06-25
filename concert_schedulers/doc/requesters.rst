Requesters
==========

Overview
--------

Requesters are generally instantiated by individual services and used internally (can be
multiple kinds of requesters in a concert, but only one scheduler). Most requesters
can be very simple and don't need special implementations.

Resource Pool Requester
-----------------------

This requester automatically manages requests that ensure a well defined pool of
resources is always available (or at least pending). A resource consists of a
resource name along with minimum and maximum requirements (e.g. ``rocon_apps/delivery`` with
a min of 2 and max of 4 as well as ``rocon_apps/cleaning`` with a min of 0 and max of 2.

This is used in conjunction with the `static link graph`_ service type.

.. _`static link graph`: http://wiki.ros.org/concert_service_link_graph
